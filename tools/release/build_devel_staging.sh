#!/usr/bin/env bash
set -ex

# LFS の endpoint が Hugging Face に移った後も access=basic が残っていると、匿名で
# 引けずに Username を聞かれる。設定が無いときは exit 5 を返すので || true で流す。
git config --unset-all lfs.https://huggingface.co/commaai/openpilot-lfs.git/info/lfs.access || true

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

SOURCE_DIR="$(git -C $DIR rev-parse --show-toplevel)"
if [ -z "$TARGET_DIR" ]; then
  TARGET_DIR="$(mktemp -d)"
fi

# set git identity
source $DIR/identity.sh

# install も --local にする。既定はグローバル(~/.gitconfig)なので、uninstall --local と
# 揃えないとグローバル側の smudge が残り、TARGET_DIR の checkout で 800MB を引いてしまう。
git lfs install --local
# LFS はアイコン・フォント・効果音・小モデルも管理しているので pull は必須。
# big_* だけは release_files.py が配布物から外すので引くだけ無駄。
git lfs pull -X "openpilot/selfdrive/modeld/models/big_*"

echo "[-] Setting up target repo T=$SECONDS"

rm -rf $TARGET_DIR
mkdir -p $TARGET_DIR
cd $TARGET_DIR
cp -r $SOURCE_DIR/.git $TARGET_DIR
pre-commit uninstall || true

echo "[-] bringing devel-staging in sync T=$SECONDS"
cd $TARGET_DIR
git branch -D devel-staging || true
git push origin --delete devel-staging || true

git checkout devel-staging
git reset --hard devel-staging

git config --local lfs.locksverify false

# LFS の upload 先(Hugging Face)は認証必須で、pre-push フックが走ると Username を
# 聞かれて止まる。push の前にフックを外しておく。
# --local を付けないとグローバル設定まで消え、SOURCE_DIR 側の smudge も効かなくなる。
git lfs uninstall --local

git push --set-upstream origin devel-staging

git fetch --depth 1 origin devel-staging

git reset --hard origin/devel-staging
git clean -xdff

# remove everything except .git
echo "[-] erasing old openpilot T=$SECONDS"
find . -maxdepth 1 -not -path './.git' -not -name '.' -not -name '..' -exec rm -rf '{}' \;

# reset source tree
cd $SOURCE_DIR
git clean -xdff

# do the files copy
echo "[-] copying files T=$SECONDS"

cd $SOURCE_DIR
# big_* は INCLUDE_BIG_MODEL が無いと release_files.py が列挙しないので、
# rsync 側で除外する必要はない。
./tools/release/release_files.py |
  rsync -l -R \
    --from0 --files-from=- \
    ./ "$TARGET_DIR/"

# in the directory
cd $TARGET_DIR
rm -f panda/board/obj/panda.bin.signed

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

git push -f origin devel-staging:devel-staging

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

