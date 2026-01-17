# How to Run in MATLAB Online

## 🔗 Quick Access Link
**Direct Project Access**: https://drive.mathworks.com/sharing/6aab9a91-d00f-461e-b031-61f1169b1f2d

*Click the link above to open the complete project directly in MATLAB Online with all files ready to run.*

---

## Step 1: Access MATLAB Online
1. Go to: https://matlab.mathworks.com/
2. Sign in with your account
3. **OR** use the direct link above to access the shared project

## Step 2: Upload the Script
1. Click the **Upload** button (top toolbar)
2. Navigate to: `D:\Masters\Robotics\mini_project\MATLAB_ONLINE_RUNNER.m`
3. Upload the file

## Step 3: Run the Script
1. In MATLAB Online, open `MATLAB_ONLINE_RUNNER.m`
2. Click **Run** button (green play button) or press `F5`
3. Wait for execution (should take 10-30 seconds)

## Step 4: Download Results
After execution completes, you'll see:
- ✓ Test results in the Command Window
- ✓ Three PNG figure files created:
  - `Figure1_Pentagon_Path.png`
  - `Figure2_Linear_Trajectory.png`
  - `Figure3_SLERP_Orientation.png`

To download the figures:
1. Right-click each PNG file in the file browser
2. Select **Download**
3. Save to your computer

## Expected Output

```
==========================================================
       ROBOTICS PROJECT - MATLAB ONLINE RUNNER           
==========================================================

Running Quaternion Tests...

TEST 1: Identity Quaternion
  Error: 0.0000000000e+00
  Result: PASS ✓

TEST 2: 90° Z-axis Rotation
  Error: 1.0000000000e-06
  Result: PASS ✓

TEST 3: Rotation Matrix Properties
  Determinant: 1.0000000000
  Orthogonality: 8.88e-16
  Result: PASS ✓

Generating Linear Trajectory...

Trajectory Generated:
  Points: 20
  Path length: 0.4123 m
  Linearity deviation: 0.0000000000e+00 m
  Result: PASS ✓

Creating Figure 1: Pentagon Path...
  ✓ Saved: Figure1_Pentagon_Path.png

Creating Figure 2: Linear Trajectory...
  ✓ Saved: Figure2_Linear_Trajectory.png

Creating Figure 3: SLERP Orientation...
  ✓ Saved: Figure3_SLERP_Orientation.png

==========================================================
                    EXECUTION SUMMARY                     
==========================================================
✓ All quaternion tests completed
✓ Linear trajectory generated and verified
✓ 3 figures created and saved:
  - Figure1_Pentagon_Path.png
  - Figure2_Linear_Trajectory.png
  - Figure3_SLERP_Orientation.png

All files are ready for download from MATLAB Online!
==========================================================
```

## Troubleshooting

**If you get errors:**
- Make sure you're using MATLAB R2019b or later
- The script is self-contained (no additional files needed)
- All helper functions are included at the bottom

**If figures don't display:**
- They are still saved as PNG files
- Check the "Current Folder" panel on the left
- Right-click to download

## Alternative: Copy-Paste Method

If upload doesn't work:
1. Open the file `MATLAB_ONLINE_RUNNER.m` in a text editor
2. Copy the entire contents (Ctrl+A, Ctrl+C)
3. In MATLAB Online, click **New Script**
4. Paste the code (Ctrl+V)
5. Save as `MATLAB_ONLINE_RUNNER.m`
6. Click **Run**

---

**File Location**: `D:\Masters\Robotics\mini_project\MATLAB_ONLINE_RUNNER.m`
**Size**: ~8 KB
**Execution Time**: 10-30 seconds
