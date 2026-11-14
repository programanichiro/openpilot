// イチロウパイロット用インストーラ配信
//   <名前>.workers.dev/<ブランチ名>  で任意のブランチのインストーラを返す
//
// 仕組み: comma が配っているインストーラ ELF を取得し、中に埋め込まれた
// リポジトリURLとブランチ名を書き換えて返す。installer.cc がこの2つを
// 「文字列 + '?' + 空白パディング」という固定長領域として持っているので
// (ソースに「Leave some extra space for the fork installer」とある)、
// 全体の長さを変えずに差し替えられる。
//
// C4(mici) では comma のサーバが commaai 以外の user を 409 で弾くため、
// この経路を挟むことで fork のインストールを可能にする。

const REPO_URL = "https://github.com/programanichiro/openpilot.git";
const TEMPLATE = "https://installer.comma.ai/commaai/nightly-dev";

// 固定長領域を書き換える。old の直後が '?' + 空白の並びになっている。
function patch(buf, oldStr, newStr, label) {
  const enc = new TextEncoder();
  const needle = enc.encode(oldStr + "?");
  const at = indexOf(buf, needle);
  if (at < 0) throw new Error(`${label}: 領域が見つからない`);

  // '?' の後ろに続く空白の数を数え、領域全体の長さを得る
  let pad = 0;
  let p = at + needle.length;
  while (p + pad < buf.length && buf[p + pad] === 0x20) pad++;
  const total = needle.length + pad;          // '?' と空白を含む
  const value = enc.encode(newStr);
  if (value.length > total - 1) throw new Error(`${label}: 長すぎる`);

  buf.set(value, at);
  buf[at + value.length] = 0x3f;              // '?'
  buf.fill(0x20, at + value.length + 1, at + total);
}

function indexOf(hay, needle) {
  outer: for (let i = 0; i + needle.length <= hay.length; i++) {
    for (let j = 0; j < needle.length; j++) if (hay[i + j] !== needle[j]) continue outer;
    return i;
  }
  return -1;
}

// git のパケット行。先頭 4 桁は自身の長さを含む 16 進数。
function pkt(line) {
  const n = new TextEncoder().encode(line).length + 4;
  return n.toString(16).padStart(4, "0") + line;
}

// GitHub にそのブランチが実在するか確認する。git wire protocol v2 の ls-refs を
// 直接叩くので認証もレートリミットも無く、問い合わせた 1 本しか返ってこない
// (無ければ 4 バイト、あれば 70 バイト程度)。ref-prefix は前方一致なので
// release3X を訊くと release3X4 も返る。名前の完全一致をこちら側で確かめる。
// 判定できなかったときは true を返す。GitHub の一時障害でインストールを
// 止めてしまわないため、閉じる側ではなく通す側に倒す。
async function branchExists(branch) {
  const ref = `refs/heads/${branch}`;
  const body = pkt("command=ls-refs\n") + "0001" + pkt(`ref-prefix ${ref}\n`) + "0000";
  try {
    const r = await fetch(REPO_URL + "/git-upload-pack", {
      method: "POST",
      headers: {
        "User-Agent": "git/2.39.0",
        "Git-Protocol": "version=2",
        "Content-Type": "application/x-git-upload-pack-request",
      },
      body,
    });
    if (!r.ok) return true;
    return (await r.text()).includes(` ${ref}\n`);
  } catch {
    return true;
  }
}

export default {
  async fetch(request) {
    const url = new URL(request.url);
    const path = url.pathname.replace(/^\/+|\/+$/g, "");

    // ブランチ名が無いとどのブランチを入れたいのか決まらない。既定値を持たせると
    // 意図しないブランチを入れてしまうので、指定を促して終わる。
    if (path === "") {
      return new Response("specify a branch name\n", {
        status: 400,
        headers: { "Content-Type": "text/plain; charset=utf-8" },
      });
    }

    const branch = decodeURIComponent(path);

    // ブランチ名として妥当な文字だけ許可する
    if (!/^[\w.\-/]{1,60}$/.test(branch)) {
      return new Response("invalid branch name\n", { status: 400 });
    }

    // 打ち間違えたブランチ名のインストーラを渡すと、実機で git clone が失敗して
    // インストーラが無言で落ちる。原因が分かるようにここで弾く。
    if (!(await branchExists(branch))) {
      return new Response(`branch not found (${branch})\n`, {
        status: 404,
        headers: { "Content-Type": "text/plain; charset=utf-8" },
      });
    }

    // comma のテンプレートを取得。AGNOSSetup の User-Agent でないと弾かれる。
    const ua = request.headers.get("User-Agent") || "";
    const res = await fetch(TEMPLATE, {
      headers: {
        "User-Agent": ua.startsWith("AGNOSSetup") ? ua : "AGNOSSetup-19.9",
        "X-openpilot-device-type": request.headers.get("X-openpilot-device-type") || "mici",
      },
    });
    if (!res.ok) return new Response(`template fetch failed: ${res.status}\n`, { status: 502 });

    const buf = new Uint8Array(await res.arrayBuffer());
    if (!(buf[0] === 0x7f && buf[1] === 0x45 && buf[2] === 0x4c && buf[3] === 0x46)) {
      return new Response("template is not an ELF\n", { status: 502 });
    }

    try {
      patch(buf, "https://github.com/commaai/openpilot.git", REPO_URL, "GIT_URL");
      // テンプレートのブランチ名は版によって変わるので候補を順に試す
      const cands = ["nightly-dev", "nightly", "release3-staging", "release3"];
      const hit = cands.find((c) => indexOf(buf, new TextEncoder().encode(c + "?")) >= 0);
      if (!hit) throw new Error("BRANCH: 領域が見つからない");
      patch(buf, hit, branch, "BRANCH");
    } catch (e) {
      return new Response(`patch failed: ${e.message}\n`, { status: 500 });
    }

    return new Response(buf, {
      headers: {
        "Content-Type": "application/octet-stream",
        "Content-Length": String(buf.length),
        "Cache-Control": "no-store",
      },
    });
  },
};
