#!/usr/bin/env bash
set -ex

# キーが無いと終了コード5を返すので、set -e で落ちないようにする
git config --unset-all lfs.https://huggingface.co/commaai/openpilot-lfs.git/info/lfs.access || true

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

SOURCE_DIR="$(git -C $DIR rev-parse --show-toplevel)"
if [ -z "$TARGET_DIR" ]; then
  TARGET_DIR="$(mktemp -d)"
fi

# set git identity
source $DIR/identity.sh

git lfs update --force
git lfs install
# ビッグモデルは 737MB あり LFS のまま配る。実体は不要なのでポインタのまま残す。
git lfs pull -X "openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl"

echo "[-] Setting up target repo T=$SECONDS"

rm -rf $TARGET_DIR
mkdir -p $TARGET_DIR
cd $TARGET_DIR
cp -r $SOURCE_DIR/.git $TARGET_DIR
pre-commit uninstall || true

echo "[-] bringing __nightly-chestnut in sync T=$SECONDS"
cd $TARGET_DIR
git branch -D __nightly-chestnut || true
git push origin --delete __nightly-chestnut || true

# origin と remotes(公式) の両方に同名の追跡 ref があると DWIM が曖昧になるので upstream を明示する。
# origin 側の追跡 ref は push --delete が失敗したときに残骸として残ることがある。
git checkout -B __nightly-chestnut remotes/__nightly-chestnut

git config --local lfs.locksverify false

# LFS の upload 先(Hugging Face)は認証必須で、pre-push フックが走ると Username を聞かれて止まる。
# このブランチでは LFS を使わず、ビッグモデルはポインタのままコミットするので先に外しておく。
git lfs uninstall

git push --set-upstream origin __nightly-chestnut

git fetch --depth 1 origin __nightly-chestnut

git reset --hard origin/__nightly-chestnut
git clean -xdff

# ----------------------------------------
# .gitattributes を退避
# ----------------------------------------

# release_files.py は .gitattributes を除外するので、ブランチにあるものを持ち回す。起点が upstream の
# __nightly-chestnut なので、LFS 指定はビッグモデルの1行だけになっている。
GITATTR_BACKUP=$(mktemp -d)
cp .gitattributes $GITATTR_BACKUP/

# remove everything except .git
echo "[-] erasing old openpilot T=$SECONDS"
find . -maxdepth 1 -not -path './.git' -not -name '.' -not -name '..' -exec rm -rf '{}' \;

# reset source tree
cd $SOURCE_DIR
git clean -xdff

# do the files copy
echo "[-] copying files T=$SECONDS"

cd $SOURCE_DIR
#cp -pR --parents $(./tools/release/release_files.py) $TARGET_DIR/
#rsync -l -R --exclude='big_driving_*.onnx' $(./tools/release/release_files.py) $TARGET_DIR/
INCLUDE_BIG_MODEL=1 ./tools/release/release_files.py |
  rsync -l -R \
    --from0 --files-from=- \
    ./ "$TARGET_DIR/"

# in the directory
cd $TARGET_DIR
rm -f panda/board/obj/panda.bin.signed

# ----------------------------------------
# .gitattributes を復元
# ----------------------------------------

cp $GITATTR_BACKUP/.gitattributes .
rm -rf $GITATTR_BACKUP

# include source commit hash and build date in commit
GIT_HASH=$(git --git-dir=$SOURCE_DIR/.git rev-parse HEAD)
DATETIME=$(date '+%Y-%m-%dT%H:%M:%S')
VERSION=$(cat $SOURCE_DIR/openpilot/common/version.h | awk -F\" '{print $2}')

echo "[-] committing version $VERSION T=$SECONDS"
git add -f .
git status
git commit -a -m "openpilot v$VERSION release-pi

date: $DATETIME
master commit: $GIT_HASH
"

git push -f origin __nightly-chestnut:__nightly-chestnut

echo "[-] done T=$SECONDS, ready at $TARGET_DIR"

if [ -n "$1" ]; then
  git branch -D $1 || true
  git push origin --delete $1 || true

  git checkout -b $1
  git push --set-upstream origin $1
  echo "add checkout : $1"
else
  echo "done!!"
fi

