# Oversized files

GitHub normal Git rejects individual blobs of 100 MB or larger. Files from `/home/seeed/ros2` that exceed this limit are stored here as 90 MiB split parts named `original_name.part-00000`, `original_name.part-00001`, ... plus `original_name.sha256`.

To restore one file, run from the repository root:

```bash
base="path/to/original_file"
cat "$base".part-* > "$base"
sha256sum -c "$base.sha256"
```

To restore all split files:

```bash
find . -name '*.sha256' -print0 | while IFS= read -r -d '' sum; do
  base="${sum#./}"
  base="${base%.sha256}"
  cat "$base".part-* > "$base"
  sha256sum -c "$sum"
done
```
