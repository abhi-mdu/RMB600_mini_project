# 📝 MATLAB Online Upload Checklist

Use this checklist to ensure you upload all necessary files to MATLAB Online.

## ✅ Upload Checklist

### Phase 1: MATLAB Scripts (4 files)
- [ ] `robot_simulation.m` - Main simulation
- [ ] `test_functions.m` - Test suite
- [ ] `compare_movej_movel.m` - Comparison tool
- [ ] `generate_media_online.m` - Automated generation script

**How to upload**:
1. Go to https://matlab.mathworks.com
2. Click **Upload** button (top toolbar)
3. Select all 4 .m files
4. Click Open

---

### Phase 2: Robot Folder Structure

#### Step 1: Create `robot` folder
- [ ] Right-click in file browser → New Folder
- [ ] Name it: `robot`
- [ ] Double-click to enter folder

#### Step 2: Upload URDF file
- [ ] Upload `test.urdf` into `robot/` folder

**File location on your PC**: `d:\Masters\Robotics\mini_project\robot\test.urdf`

#### Step 3: Create `IRB1600` subfolder
- [ ] Inside `robot/` folder, create new folder
- [ ] Name it: `IRB1600`
- [ ] Double-click to enter folder

#### Step 4: Upload STL files (7 files)
Inside `robot/IRB1600/` folder, upload:
- [ ] `base.stl`
- [ ] `link1.stl`
- [ ] `link2.stl`
- [ ] `link3.stl`
- [ ] `link4.stl`
- [ ] `link5.stl`
- [ ] `link6.stl`

**File location on your PC**: `d:\Masters\Robotics\mini_project\robot\IRB1600\*.stl`

---

## 📂 Final Structure on MATLAB Online

After uploading, your MATLAB Online file structure should look like:

```
MATLAB Drive/
├── robot_simulation.m
├── test_functions.m
├── compare_movej_movel.m
├── generate_media_online.m
└── robot/
    ├── test.urdf
    └── IRB1600/
        ├── base.stl
        ├── link1.stl
        ├── link2.stl
        ├── link3.stl
        ├── link4.stl
        ├── link5.stl
        └── link6.stl
```

**Total: 11 files uploaded**

---

## ✅ Verification

After uploading, verify in MATLAB Online command window:

```matlab
% Check files exist
files_to_check = {
    'robot_simulation.m'
    'test_functions.m'
    'compare_movej_movel.m'
    'generate_media_online.m'
    'robot/test.urdf'
    'robot/IRB1600/base.stl'
    'robot/IRB1600/link1.stl'
    'robot/IRB1600/link2.stl'
    'robot/IRB1600/link3.stl'
    'robot/IRB1600/link4.stl'
    'robot/IRB1600/link5.stl'
    'robot/IRB1600/link6.stl'
};

fprintf('Checking files...\n');
all_present = true;
for i = 1:length(files_to_check)
    if exist(files_to_check{i}, 'file')
        fprintf('  [OK] %s\n', files_to_check{i});
    else
        fprintf('  [MISSING] %s\n', files_to_check{i});
        all_present = false;
    end
end

if all_present
    fprintf('\n[SUCCESS] All files present!\n');
    fprintf('Ready to run: generate_media_online\n');
else
    fprintf('\n[WARNING] Some files missing. Please upload them.\n');
end
```

---

## 🚀 Ready to Execute?

Once all checkboxes are checked:

```matlab
% Run the automated generation
generate_media_online
```

This will generate 6 figures automatically in ~5-10 minutes.

---

## 📥 Download Checklist (After Execution)

After running all scripts, download these files:

### Automatically Generated (6 files):
- [ ] `fig1_error_message.png`
- [ ] `fig2_quaternion_visualization.png`
- [ ] `fig3_test_results.png`
- [ ] `fig5_slerp_comparison.png`
- [ ] `fig8_frame_hierarchy.png`
- [ ] `fig10_performance_chart.png`

### Manually Captured (4 files):
- [ ] `fig4_movel_path.png` (from robot_simulation)
- [ ] `fig6_movej_path.png` (from robot_simulation)
- [ ] `fig7_comparison.png` (from compare_movej_movel)
- [ ] `fig9_pentagon_path.png` (from robot_simulation)

### Optional Videos (3 files):
- [ ] `video1_full_simulation.mp4`
- [ ] `video2_comparison.mp4`
- [ ] `video3_tests.mp4`

---

## 💡 Quick Download Method

To download all figures at once:

```matlab
% In MATLAB Online, after generating all figures
zip('all_figures.zip', 'fig*.png')
% Then download all_figures.zip
```

---

## ⏱️ Time Estimate

| Task | Time |
|------|------|
| Upload files | 5 min |
| Run generate_media_online | 10 min |
| Manual captures | 10 min |
| Download files | 5 min |
| **Total** | **30 min** |

---

## 🎯 Success Criteria

You're ready to move to local presentation creation when:

- ✅ All 11 files uploaded to MATLAB Online
- ✅ generate_media_online ran successfully
- ✅ 6 automated figures generated
- ✅ 4 manual figures captured
- ✅ All 10 PNG files downloaded to local machine
- ✅ Files placed in `d:\Masters\Robotics\mini_project\figures\`
- ✅ `.\verify_media.ps1` shows "[SUCCESS]"

---

**Next**: See MATLAB_ONLINE_GUIDE.md for complete execution instructions.
