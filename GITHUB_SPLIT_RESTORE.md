# GitHub split-file restore notes

GitHub normal Git rejects individual blobs of 100 MB or larger. Source files from `/home/seeed/ros2` that exceed that limit are stored in this repository as 90 MiB `*.part-*` chunks plus a matching `*.sha256` file.

Restore one split file from the repository root:

```bash
base="path/to/original_file"
cat "$base".part-* > "$base"
sha256sum -c "$base.sha256"
```
