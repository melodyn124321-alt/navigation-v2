# Restoring oversized files

GitHub rejects ordinary Git blobs of 100 MB or larger. The files listed below were split into 90 MiB parts. To restore one file, concatenate its parts in lexical order and compare the SHA-256 value in the adjacent .sha256 file.

Example:
```sh
cat path/to/file.part-* > path/to/file
sha256sum -c path/to/file.sha256
```

## bags/overwritten/raw_livox_manual_001_20260727_205145/raw_livox_manual_001_0.db3

- Size: 1052790784 bytes
- SHA-256: b42af34ac1453a8190a7337ce521b6df8689d566b370171c609c3cd9a399e481
- Parts: bags/overwritten/raw_livox_manual_001_20260727_205145/raw_livox_manual_001_0.db3.part-*

## bags/overwritten/raw_livox_manual_001_20260729_174143/raw_livox_manual_001_0.db3

- Size: 1343086592 bytes
- SHA-256: 63d8b6e8931e3aaf9acb5ffce04cac25cbe7fa37c57787402bcf89a5f34a3d89
- Parts: bags/overwritten/raw_livox_manual_001_20260729_174143/raw_livox_manual_001_0.db3.part-*

## bags/overwritten/raw_livox_manual_001_20260729_200219/raw_livox_manual_001_0.db3

- Size: 1280831488 bytes
- SHA-256: c3bc07c86f42af207bb75eea37981dfb3de4b781c5d080609bfa7308041c1614
- Parts: bags/overwritten/raw_livox_manual_001_20260729_200219/raw_livox_manual_001_0.db3.part-*

## bags/overwritten/raw_livox_manual_003_20260801_183723/raw_livox_manual_003_0.db3

- Size: 685387776 bytes
- SHA-256: ef47afa7eb014db191037873159df15e8033d2200024cf5e281495f1a3243ad1
- Parts: bags/overwritten/raw_livox_manual_003_20260801_183723/raw_livox_manual_003_0.db3.part-*

## bags/overwritten/raw_livox_manual_005_20260803_190440/raw_livox_manual_005_0.db3

- Size: 1858625536 bytes
- SHA-256: 8a3f357cc46b3d3d322f2479ecc89251f8a1662770ec5997dc173037f724dcf5
- Parts: bags/overwritten/raw_livox_manual_005_20260803_190440/raw_livox_manual_005_0.db3.part-*

## bags/overwritten/raw_livox_manual_005_20260803_192149/raw_livox_manual_005_0.db3

- Size: 3187642368 bytes
- SHA-256: 293870bea2e87ee23bb153508cf8f13327ea068d70d5036bdbc9f1275400fd29
- Parts: bags/overwritten/raw_livox_manual_005_20260803_192149/raw_livox_manual_005_0.db3.part-*

## bags/raw_livox_manual_001/raw_livox_manual_001_0.db3

- Size: 1207390208 bytes
- SHA-256: eb83868e8915fc44fe79a017e1b9752a69db2cdf4550f6d91c9ff196bff3a3a6
- Parts: bags/raw_livox_manual_001/raw_livox_manual_001_0.db3.part-*

## bags/raw_livox_manual_002/raw_livox_manual_002_0.db3

- Size: 4737695744 bytes
- SHA-256: 02f024b29f1938e15328fb1f66f337c9df055c154bad1acab90b5a313c3807fe
- Parts: bags/raw_livox_manual_002/raw_livox_manual_002_0.db3.part-*

## bags/raw_livox_manual_003/raw_livox_manual_003_0.db3

- Size: 4207804416 bytes
- SHA-256: f5f8ebe810ff0cc9f337285dd3dc78b854d7d35e9f18c72924eccab0c9630d71
- Parts: bags/raw_livox_manual_003/raw_livox_manual_003_0.db3.part-*

## bags/raw_livox_manual_004/raw_livox_manual_004_0.db3

- Size: 6767153152 bytes
- SHA-256: 2895155578efd54d1b906c3e766890224a3e8d6349964a582bcf88129c3fef9b
- Parts: bags/raw_livox_manual_004/raw_livox_manual_004_0.db3.part-*

## bags/raw_livox_manual_005/raw_livox_manual_005_0.db3

- Size: 2795073536 bytes
- SHA-256: f9cb67932dbd82f248e7f0816d003ddb7280e0c4bc01c8989dab48264b1aa550
- Parts: bags/raw_livox_manual_005/raw_livox_manual_005_0.db3.part-*

## bags/raw_livox_selfcheck_20260727_01/raw_livox_selfcheck_20260727_01_0.db3

- Size: 119029760 bytes
- SHA-256: 8367d96c4b8ecb8a3f747939a0372011cfa006534bb7bfc93772b3be7597defa
- Parts: bags/raw_livox_selfcheck_20260727_01/raw_livox_selfcheck_20260727_01_0.db3.part-*

## env/Sophus/build/test/core/CMakeFiles/test_rxso3.dir/test_rxso3.cpp.o

- Size: 147672808 bytes
- SHA-256: d51b6572f0aba009dcbc2d69c2a390e5dfd626fa7e2eb458efa0972d000eb703
- Parts: env/Sophus/build/test/core/CMakeFiles/test_rxso3.dir/test_rxso3.cpp.o.part-*

## env/Sophus/build/test/core/CMakeFiles/test_se3.dir/test_se3.cpp.o

- Size: 135395584 bytes
- SHA-256: 04b3c77d351d35909d201bf1ebe4f710dff8487f9c88c11f0eab2a8645e8454e
- Parts: env/Sophus/build/test/core/CMakeFiles/test_se3.dir/test_se3.cpp.o.part-*

## env/Sophus/build/test/core/CMakeFiles/test_sim3.dir/test_sim3.cpp.o

- Size: 122682264 bytes
- SHA-256: 2c2affa7576a5950ba343826c001d3634f0037049732901486b75323a32ac9a7
- Parts: env/Sophus/build/test/core/CMakeFiles/test_sim3.dir/test_sim3.cpp.o.part-*

## env/Sophus/build/test/core/CMakeFiles/test_so3.dir/test_so3.cpp.o

- Size: 145002880 bytes
- SHA-256: 8e0957fd3b197f81d5c17953dde01f256db9f019d784707d6964ace9ee6dbb9b
- Parts: env/Sophus/build/test/core/CMakeFiles/test_so3.dir/test_so3.cpp.o.part-*

## maps/replay/fastlio_map_manual_002_raw.pcd

- Size: 216676445 bytes
- SHA-256: 4cb0d6700d90cd715703a8fec1c4575f98f33f5c65fd9f121bdfa8bef377f99a
- Parts: maps/replay/fastlio_map_manual_002_raw.pcd.part-*

## maps/replay/fastlio_map_manual_003_raw.pcd

- Size: 192289053 bytes
- SHA-256: f29dcc8c63de534b58bf086cb1156c795b9b98752521cd39d49f934a9c695b12
- Parts: maps/replay/fastlio_map_manual_003_raw.pcd.part-*

## maps/replay/fastlio_map_manual_004_raw.pcd

- Size: 310225661 bytes
- SHA-256: 9c27e96ddc1084cb674cdc15f23205dd97fdc949305058d2edf1c1f554298db5
- Parts: maps/replay/fastlio_map_manual_004_raw.pcd.part-*

## maps/replay/fastlio_map_manual_005_raw.pcd

- Size: 106309917 bytes
- SHA-256: 5f1ee018919714fe252d1e1cfdd673c2000881337c2d0088f650c65acd0d94cf
- Parts: maps/replay/fastlio_map_manual_005_raw.pcd.part-*

