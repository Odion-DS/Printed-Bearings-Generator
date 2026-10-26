# Printed Bearings Generator

A Fusion 360 add-in for generating parametric 3D-printable ball bearings with customizable dimensions.

## Features

- **Parametric Design**: Fully adjustable bearing dimensions
- **Complete Bearing Assembly**: Generates inner ring, outer ring, and cage
- **Optional Ball Generation**: Create ball bearings with circular pattern
- **Automatic Grouping**: All features organized in timeline groups
- **Named Bodies**: All components automatically named with dimensions

## Installation

1. Copy the `PrintedBearings` folder to:
   ```
   %APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\
   ```

2. In Fusion 360, go to **Tools > Add-Ins > Scripts and Add-Ins**

3. Select the **Scripts** tab

4. Find **PrintedBearings** in the list and click **Run**

## Usage

### Parameters

- **Create as component**: Creates the bearing as a separate component
- **Create balls**: Generates ball bearings (optional, default: off)
- **Ball size**: Diameter of the ball bearings (mm)
- **Ball clearance ring**: Clearance between ball and ring race (mm)
- **Ball clearance cage**: Clearance between ball and cage (mm)
- **Ball count**: Number of balls in the bearing
- **Inner diameter**: Inner diameter of the bearing (mm)
- **Top/bottom wall thickness**: Vertical wall thickness (mm)
- **Left/right wall thickness**: Radial wall thickness (mm)

### Output

The script generates:

1. **Inner_Ring_{diameter}mm**: Inner bearing ring with ball race
2. **Outer_Ring_{diameter}mm**: Outer bearing ring with ball race
3. **Cage_{diameter}mm**: Ball cage with cutouts for balls
4. **Ball_{size}mm** (optional): Ball bearings in circular pattern

All features are grouped in a timeline group named `Printed_Bearing_{diameter}mm`.

## Design Details

### Ring Design
- Two rings (inner and outer) created via revolution on XZ plane
- Ball race groove sized to ball diameter + clearance
- Configurable wall thickness for strength

### Cage Design
- Annular ring extruded from XY plane
- Height: 1/2 total height + 1/3 ball diameter
- Ball cutouts created via circular pattern
- Positioned at -1/2 height on Z axis

### Ball Design
- Spheres created via revolution
- Positioned at average of cage inner/outer radius
- Distributed evenly in circular pattern

## Formulas

### Cage Dimensions
```
Inner diameter = inner_diameter + 2*wall_lr + 1.5*ball_size - 2*ball_clearance_cage
Outer diameter = inner_diameter + 2*wall_lr + 0.5*ball_size + ball_size/2 - 2*ball_clearance_cage
Extrude height = 1/2 * total_height + 1/3 * ball_size
```

### Display Values
```
Outer diameter = 2*(ball_size + 2*wall_lr) + inner_diameter
Total height = ball_size + 2*wall_tb
```

## Tips for 3D Printing

1. **Clearances**: Adjust clearances based on your printer's tolerance
   - Start with 0.05mm for ring clearance
   - Use 0.1mm for cage clearance
   - Test with small bearings first

2. **Brim**: A brim is recommended if you have adhesion problems with the small parts

3. **Tested Sizes**: Works well down to 4mm ball bearings

## Requirements

- Autodesk Fusion 360
- Python API support

## License

MIT License - See LICENSE file for details.

## Version

1.0 - Initial release
