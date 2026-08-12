Oversize file restore note
==========================

GitHub LFS rejects individual files larger than 2 GiB. The source file below
was 2,607,337,472 bytes, so it is stored in this repository as split parts:

Original source path:

  .git.bak/lfs/objects/2d/2e/2d2e4632ffd8878e3088464c2546ae265f318f6d4030311bca535347ae4902c6

Uploaded parts:

  .git.bak/lfs/objects/2d/2e/2d2e4632ffd8878e3088464c2546ae265f318f6d4030311bca535347ae4902c6.part-00
  .git.bak/lfs/objects/2d/2e/2d2e4632ffd8878e3088464c2546ae265f318f6d4030311bca535347ae4902c6.part-01
  .git.bak/lfs/objects/2d/2e/2d2e4632ffd8878e3088464c2546ae265f318f6d4030311bca535347ae4902c6.sha256

Restore command from the repository root:

  cat .git.bak/lfs/objects/2d/2e/2d2e4632ffd8878e3088464c2546ae265f318f6d4030311bca535347ae4902c6.part-* > .git.bak/lfs/objects/2d/2e/2d2e4632ffd8878e3088464c2546ae265f318f6d4030311bca535347ae4902c6
  sha256sum -c .git.bak/lfs/objects/2d/2e/2d2e4632ffd8878e3088464c2546ae265f318f6d4030311bca535347ae4902c6.sha256
