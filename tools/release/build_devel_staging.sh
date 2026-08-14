#!/usr/bin/env bash
set -ex

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

SOURCE_DIR="$(git -C $DIR rev-parse --show-toplevel)"
if [ -z "$TARGET_DIR" ]; then
  TARGET_DIR="$(mktemp -d)"
fi

# set git identity
source $DIR/identity.sh

git lfs install
git lfs pull

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

git push --set-upstream origin devel-staging

git fetch --depth 1 origin devel-staging

git reset --hard origin/devel-staging
git clean -xdff
git lfs uninstall

# ----------------------------------------
# backup chunked model files
# ----------------------------------------

#MODEL_BACKUP=$(mktemp -d)

#cp openpilot/selfdrive/modeld/models/big_driving_*.onnx.chunk* $MODEL_BACKUP/

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
rsync -l -R --exclude='big_driving_*.onnx' $(./tools/release/release_files.py) $TARGET_DIR/

# in the directory
cd $TARGET_DIR
rm -f panda/board/obj/panda.bin.signed

# ----------------------------------------
# restore chunked model files
# ----------------------------------------

mkdir -p openpilot/selfdrive/modeld/models

#cp $MODEL_BACKUP/* openpilot/selfdrive/modeld/models/

#rm -f openpilot/selfdrive/modeld/models/big_driving_*.onnx

# remove accidental git index entry
#git rm --cached openpilot/selfdrive/modeld/models/big_driving_*.onnx || true

# ensure chunks are tracked
#git add openpilot/selfdrive/modeld/models/big_driving_*.onnx.chunk*

#rm -rf $MODEL_BACKUP

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

