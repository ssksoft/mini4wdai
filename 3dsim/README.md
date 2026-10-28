# 3dsim — Mini 4WD 3D simulation (Unreal Engine 5.8)

This is the 3D simulation environment for the Tamiya Oval Home Circuit (two-level lane change type, 69569) and the Mad Racer (MA chassis).

```
3dsim/
  MiniYonkuSim/        Unreal Engine project (open MiniYonkuSim.uproject)
    Content/Mini4WD/   course parts, car, materials, map Maps/Mini4WD_Oval
    Content/Python/    init_unreal.py (auto-loads the keyboard drive script at startup)
  mini4wd_course/      generation scripts and source data (see README there)
    generator/         Python that builds the course/car FBX from dimensions (track_params.json, build_car.py)
    ue/                UE editor scripts (setup_in_ue.py, car_setup.py, m4wd_drive.py, ...)
    fbx/ car/          generated course parts and car FBX
    scan_fbx/          textured visuals cleaned from the 3D scans
```

## Open and drive

1. Double-click `MiniYonkuSim/MiniYonkuSim.uproject` to open it in UE 5.8.
2. Open `Content/Mini4WD/Maps/Mini4WD_Oval` and press ▶ (Play).
3. Controls: W accelerate / S brake / R reset / 1, 2 lane / C camera / M motor. See `mini4wd_course/README.md` for details.

The scripts find their paths relative to the project, so the repository works wherever it is cloned.

## Git usage

Binary files such as `.uasset` / `.umap` / `.fbx` / images are managed with **Git LFS** (`3dsim/.gitattributes`).
UE-generated folders (`Binaries`, `Intermediate`, `Saved`, `DerivedDataCache`) are excluded (`3dsim/.gitignore`).

First time only (per PC):
```
git lfs install
```

Usual flow:
```
git add 3dsim
git commit -m "3dsim: ..."
git push
```

- **Before committing:** save in UE (Ctrl+Shift+S).
- **Binary assets:** they can't be merged, so if several people edit the same asset, take a lock with `git lfs lock <file>`.
- **Changing course or car dimensions:** edit `mini4wd_course/generator/*`, regenerate the FBX, then re-import in UE (see `mini4wd_course/README.md`).
