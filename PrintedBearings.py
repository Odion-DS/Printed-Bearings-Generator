"""Printed bearing generator (first step)."""

import math
import traceback
import adsk.core
import adsk.fusion

app = adsk.core.Application.get()
ui = app.userInterface

CMD_ID = 'printed_bearing_cmd'
CMD_NAME = 'Printed Bearing'
CMD_DESC = 'Create a printed bearing base geometry.'

handlers = []


def mm_to_cm(value_mm: float) -> float:
    return value_mm / 10.0


def cm_to_mm(value_cm: float) -> float:
    return value_cm * 10.0


def pick_side_profile(
    profiles: adsk.fusion.Profiles,
    x_min_cm: float,
    x_max_cm: float
) -> adsk.fusion.Profile | None:
    best_profile = None
    best_area = 0.0

    for i in range(profiles.count):
        profile = profiles.item(i)
        props = profile.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        centroid = props.centroid
        if x_min_cm <= centroid.x <= x_max_cm and props.area > best_area:
            best_area = props.area
            best_profile = profile

    return best_profile


def input_len_mm(inputs: adsk.core.CommandInputs, input_id: str) -> float:
    value_input = adsk.core.ValueCommandInput.cast(inputs.itemById(input_id))
    return cm_to_mm(value_input.value)


def update_display_values(inputs: adsk.core.CommandInputs):
    ball_size = input_len_mm(inputs, 'ballSize')
    inner_diameter = input_len_mm(inputs, 'innerDiameter')
    wall_lr = input_len_mm(inputs, 'wallLR')
    wall_tb = input_len_mm(inputs, 'wallTB')

    outer_diameter = 2.0 * (ball_size + 2.0 * wall_lr) + inner_diameter
    total_height = ball_size + 2.0 * wall_tb

    outer_text = adsk.core.TextBoxCommandInput.cast(inputs.itemById('outerDiameterDisplay'))
    height_text = adsk.core.TextBoxCommandInput.cast(inputs.itemById('heightDisplay'))
    outer_text.text = f'{outer_diameter:.3f} mm'
    height_text.text = f'{total_height:.3f} mm'


def create_bearing_geometry(
    design: adsk.fusion.Design,
    create_as_component: bool,
    ball_size: float,
    ball_clearance_ring: float,
    ball_clearance_cage: float,
    ball_count: int,
    inner_diameter: float,
    wall_tb: float,
    wall_lr: float,
    create_balls: bool,
    close_cage_ratio: float
):
    root_comp = design.rootComponent
    target_comp = root_comp

    if create_as_component:
        occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        target_comp = occ.component
        target_comp.name = 'Printed Bearing'

    _ball_count = max(1, int(ball_count))
    _ball_clearance_cage = ball_clearance_cage

    inner_radius = inner_diameter / 2.0
    ball_radius = ball_size / 2.0
    radial_span = 2.0 * wall_lr + ball_size
    outer_radius = inner_radius + radial_span
    total_height = ball_size + 2.0 * wall_tb
    half_height = total_height / 2.0
    side_width = wall_lr + 0.25 * ball_size

    sketches = target_comp.sketches
    section_sketch = sketches.add(target_comp.xZConstructionPlane)
    section_sketch.name = 'Bearing Section'

    lines = section_sketch.sketchCurves.sketchLines
    circles = section_sketch.sketchCurves.sketchCircles

    def p_xz(x_mm: float, z_mm: float) -> adsk.core.Point3D:
        return adsk.core.Point3D.create(mm_to_cm(x_mm), mm_to_cm(z_mm), 0)

    axis_line = lines.addByTwoPoints(p_xz(0, -total_height), p_xz(0, total_height))
    axis_line.isConstruction = True

    inner_radius_line = lines.addByTwoPoints(p_xz(0, 0), p_xz(inner_radius, 0))
    inner_radius_line.isConstruction = True

    span_line = lines.addByTwoPoints(p_xz(inner_radius, 0), p_xz(outer_radius, 0))
    span_line.isConstruction = True

    # Left ring rectangle.
    lines.addByTwoPoints(p_xz(inner_radius, -half_height), p_xz(inner_radius, half_height))
    lines.addByTwoPoints(p_xz(inner_radius, half_height), p_xz(inner_radius + side_width, half_height))
    lines.addByTwoPoints(p_xz(inner_radius + side_width, half_height), p_xz(inner_radius + side_width, -half_height))
    lines.addByTwoPoints(p_xz(inner_radius + side_width, -half_height), p_xz(inner_radius, -half_height))

    # Right ring rectangle.
    lines.addByTwoPoints(p_xz(outer_radius - side_width, -half_height), p_xz(outer_radius - side_width, half_height))
    lines.addByTwoPoints(p_xz(outer_radius - side_width, half_height), p_xz(outer_radius, half_height))
    lines.addByTwoPoints(p_xz(outer_radius, half_height), p_xz(outer_radius, -half_height))
    lines.addByTwoPoints(p_xz(outer_radius, -half_height), p_xz(outer_radius - side_width, -half_height))

    # Center circle connecting the two rectangles (ball path)
    race_center_x = inner_radius + (radial_span / 2.0)
    race_radius = (ball_size + ball_clearance_ring) / 2.0
    circles.addByCenterRadius(
        p_xz(race_center_x, 0),
        mm_to_cm(race_radius)
    )

    left_profile = pick_side_profile(
        section_sketch.profiles,
        mm_to_cm(inner_radius),
        mm_to_cm(inner_radius + side_width)
    )
    right_profile = pick_side_profile(
        section_sketch.profiles,
        mm_to_cm(outer_radius - side_width),
        mm_to_cm(outer_radius)
    )

    if not left_profile or not right_profile:
        raise RuntimeError('Could not find both side profiles for revolve.')

    revolves = target_comp.features.revolveFeatures
    full_angle = adsk.core.ValueInput.createByReal(2.0 * math.pi)

    # Create inner ring
    inner_rev_input = revolves.createInput(
        left_profile,
        axis_line,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    inner_rev_input.setAngleExtent(False, full_angle)
    inner_ring_feature = revolves.add(inner_rev_input)

    # Create outer ring
    outer_rev_input = revolves.createInput(
        right_profile,
        axis_line,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    outer_rev_input.setAngleExtent(False, full_angle)
    outer_ring_feature = revolves.add(outer_rev_input)

    # Cage: top-view sketch on XY plane shifted down by half height.
    planes = target_comp.constructionPlanes
    cage_plane_input = planes.createInput()
    cage_plane_input.setByOffset(
        target_comp.xYConstructionPlane,
        adsk.core.ValueInput.createByReal(mm_to_cm(-half_height))
    )
    cage_plane = planes.add(cage_plane_input)
    cage_sketch = sketches.add(cage_plane)
    cage_sketch.name = 'Cage Top'

    # Cage rings with corrected formulas.
    # Inner circle diameter: inner_diameter + 2*wall_lr + 1/2*ball_size + 2*ball_clearance_cage
    cage_inner_diameter = (
        inner_diameter
        + (2.0 * wall_lr)
        + (0.5 * ball_size)
        + (2.0 * ball_clearance_cage)
    )
    # Outer circle diameter: inner_diameter + 2*wall_lr + 0.5*ball_size + ball_size - 2*ball_clearance_cage
    cage_outer_diameter = (
        inner_diameter
        + (2.0 * wall_lr)
        + (0.5 * ball_size)
        + ball_size
        - (2.0 * ball_clearance_cage)
    )
    cage_inner_radius = cage_inner_diameter / 2.0
    cage_outer_radius = cage_outer_diameter / 2.0

    cage_circles = cage_sketch.sketchCurves.sketchCircles

    # Inner and outer cage circles on ZX plane (center at origin)
    cage_circles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0),
        mm_to_cm(cage_inner_radius)
    )
    cage_circles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0),
        mm_to_cm(cage_outer_radius)
    )

    cage_profile = None
    expected_ring_area_cm2 = math.pi * (
        (mm_to_cm(cage_outer_radius) ** 2) - (mm_to_cm(cage_inner_radius) ** 2)
    )
    best_delta = float('inf')
    for i in range(cage_sketch.profiles.count):
        profile = cage_sketch.profiles.item(i)
        props = profile.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        area_delta = abs(props.area - expected_ring_area_cm2)
        if area_delta < best_delta:
            best_delta = area_delta
            cage_profile = profile

    if not cage_profile:
        raise RuntimeError('Could not find cage annulus profile.')

    # Extrude cage upward from the offset XY plane: 1/2 height + (ball_size * close_cage_ratio)
    cage_extrude_mm = (total_height / 2.0) + (ball_size * close_cage_ratio)
    ext_input = target_comp.features.extrudeFeatures.createInput(
        cage_profile,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    ext_input.setOneSideExtent(
        adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(mm_to_cm(cage_extrude_mm))),
        adsk.fusion.ExtentDirections.PositiveExtentDirection
    )
    cage_extrude = target_comp.features.extrudeFeatures.add(ext_input)

    # Ball cutout sketch on XY plane at origin
    ball_cutout_sketch = sketches.add(target_comp.xYConstructionPlane)
    ball_cutout_sketch.name = 'Ball Cutout'

    # Circle at average of cage inner and outer radius
    avg_cage_radius = (cage_inner_radius + cage_outer_radius) / 2.0
    avg_circle = ball_cutout_sketch.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0),
        mm_to_cm(avg_cage_radius)
    )
    avg_circle.isConstruction = True

    # Small circle on left side (negative X) where the avg circle intersects X axis
    ball_cutout_radius = (ball_size + ball_clearance_cage) / 2.0
    ball_center_x = -avg_cage_radius
    ball_circle = ball_cutout_sketch.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(mm_to_cm(ball_center_x), 0, 0),
        mm_to_cm(ball_cutout_radius)
    )

    # Vertical construction line through the ball circle center (rotation axis) - NOT part of profile
    ball_axis_line = ball_cutout_sketch.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(mm_to_cm(ball_center_x), mm_to_cm(-total_height), 0),
        adsk.core.Point3D.create(mm_to_cm(ball_center_x), mm_to_cm(total_height), 0)
    )
    ball_axis_line.isConstruction = True

    # Diameter line through ball circle to split it - make it construction so it doesn't cut
    diameter_line = ball_cutout_sketch.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(mm_to_cm(ball_center_x), mm_to_cm(-ball_cutout_radius), 0),
        adsk.core.Point3D.create(mm_to_cm(ball_center_x), mm_to_cm(ball_cutout_radius), 0)
    )

    # Find the profile for the half ball circle (right half)
    ball_profile = None
    for i in range(ball_cutout_sketch.profiles.count):
        profile = ball_cutout_sketch.profiles.item(i)
        props = profile.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
        centroid = props.centroid
        # The right half of the ball circle should have centroid X > ball_center_x
        if centroid.x > mm_to_cm(ball_center_x):
            # Check if it's roughly a semicircle
            expected_area = math.pi * (mm_to_cm(ball_cutout_radius) ** 2) / 2.0
            if abs(props.area - expected_area) < expected_area * 0.5:
                ball_profile = profile
                break

    if not ball_profile:
        raise RuntimeError('Could not find ball cutout profile.')

    # Get the cage body to cut from
    cage_body = cage_extrude.bodies.item(0)

    # Revolve the half circle around the vertical axis to cut from cage ONLY
    revolve_cut_input = revolves.createInput(
        ball_profile,
        ball_axis_line,
        adsk.fusion.FeatureOperations.CutFeatureOperation
    )
    revolve_cut_input.setAngleExtent(False, full_angle)

    # Create the collection for bodies to cut
    bodies_to_cut = adsk.core.ObjectCollection.create()
    bodies_to_cut.add(cage_body)
    revolve_cut_input.participantBodies = [cage_body]

    ball_cutout_feature = target_comp.features.revolveFeatures.add(revolve_cut_input)

    # Create circular pattern of ball cutouts around the Z axis
    circular_patterns = target_comp.features.circularPatternFeatures
    z_axis = target_comp.zConstructionAxis

    input_entities = adsk.core.ObjectCollection.create()
    input_entities.add(ball_cutout_feature)

    pattern_input = circular_patterns.createInput(input_entities, z_axis)
    pattern_input.quantity = adsk.core.ValueInput.createByReal(_ball_count)
    pattern_input.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    pattern_input.isSymmetric = False

    circular_patterns.add(pattern_input)

    # Rename bodies
    inner_ring_body = inner_ring_feature.bodies.item(0)
    outer_ring_body = outer_ring_feature.bodies.item(0)
    cage_body_final = cage_extrude.bodies.item(0)

    inner_ring_body.name = f'Inner_Ring_{inner_diameter:.1f}mm'
    outer_ring_body.name = f'Outer_Ring_{inner_diameter:.1f}mm'
    cage_body_final.name = f'Cage_{inner_diameter:.1f}mm'

    # Group all features into a timeline group
    timeline = design.timeline

    # Find the start and end indices in the timeline
    start_index = -1
    end_index = -1

    for i in range(timeline.count):
        item = timeline.item(i)
        entity = item.entity

        # Find the first sketch (section_sketch) as start
        if entity == section_sketch and start_index == -1:
            start_index = i

        # Find the circular pattern as end
        if hasattr(entity, 'objectType'):
            if entity.objectType == 'adsk::fusion::CircularPatternFeature':
                # Check if this is our pattern by comparing timestamp or just use the last one
                end_index = i

    # Create balls if requested
    if create_balls:
        # Ball sketch on XY plane at origin
        ball_sketch = sketches.add(target_comp.xYConstructionPlane)
        ball_sketch.name = 'Ball'

        # Circle at average of cage inner and outer radius (construction)
        avg_cage_radius = (cage_inner_radius + cage_outer_radius) / 2.0
        ball_pos_circle = ball_sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0),
            mm_to_cm(avg_cage_radius)
        )
        ball_pos_circle.isConstruction = True

        # Ball circle on left side (negative X)
        ball_radius = ball_size / 2.0
        ball_center_x = -avg_cage_radius
        ball_circle = ball_sketch.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(mm_to_cm(ball_center_x), 0, 0),
            mm_to_cm(ball_radius)
        )

        # Vertical line through the ball circle center (rotation axis)
        ball_revolve_axis = ball_sketch.sketchCurves.sketchLines.addByTwoPoints(
            adsk.core.Point3D.create(mm_to_cm(ball_center_x), mm_to_cm(-total_height), 0),
            adsk.core.Point3D.create(mm_to_cm(ball_center_x), mm_to_cm(total_height), 0)
        )
        ball_revolve_axis.isConstruction = True

        # Diameter line to split the circle
        ball_diameter_line = ball_sketch.sketchCurves.sketchLines.addByTwoPoints(
            adsk.core.Point3D.create(mm_to_cm(ball_center_x), mm_to_cm(-ball_radius), 0),
            adsk.core.Point3D.create(mm_to_cm(ball_center_x), mm_to_cm(ball_radius), 0)
        )

        # Find the profile for the half ball circle
        ball_sphere_profile = None
        for i in range(ball_sketch.profiles.count):
            profile = ball_sketch.profiles.item(i)
            props = profile.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy)
            centroid = props.centroid
            if centroid.x > mm_to_cm(ball_center_x):
                expected_area = math.pi * (mm_to_cm(ball_radius) ** 2) / 2.0
                if abs(props.area - expected_area) < expected_area * 0.5:
                    ball_sphere_profile = profile
                    break

        if ball_sphere_profile:
            # Revolve to create sphere
            ball_rev_input = revolves.createInput(
                ball_sphere_profile,
                ball_revolve_axis,
                adsk.fusion.FeatureOperations.NewBodyFeatureOperation
            )
            ball_rev_input.setAngleExtent(False, full_angle)
            ball_feature = revolves.add(ball_rev_input)

            # Rename the ball body
            ball_body = ball_feature.bodies.item(0)
            ball_body.name = f'Ball_{ball_size:.1f}mm'

            # Create circular pattern of balls
            ball_patterns = target_comp.features.circularPatternFeatures
            z_axis_ball = target_comp.zConstructionAxis

            ball_entities = adsk.core.ObjectCollection.create()
            ball_entities.add(ball_feature)

            ball_pattern_input = ball_patterns.createInput(ball_entities, z_axis_ball)
            ball_pattern_input.quantity = adsk.core.ValueInput.createByReal(_ball_count)
            ball_pattern_input.totalAngle = adsk.core.ValueInput.createByString('360 deg')
            ball_pattern_input.isSymmetric = False

            ball_pattern_feature = ball_patterns.add(ball_pattern_input)

            # Update end_index to include balls
            for i in range(timeline.count):
                item = timeline.item(i)
                entity = item.entity
                if entity == ball_pattern_feature:
                    end_index = i
                    break

    # Create the timeline group (after all features are created)
    if start_index >= 0 and end_index >= 0 and end_index >= start_index:
        try:
            group = timeline.timelineGroups.add(start_index, end_index)
            group.name = f'Printed_Bearing_{inner_diameter:.1f}mm'
        except:
            pass  # Timeline grouping failed, but continue


class BearingCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox('No active Fusion design found.')
                return

            cmd = adsk.core.Command.cast(args.firingEvent.sender)
            inputs = cmd.commandInputs

            create_as_component = adsk.core.BoolValueCommandInput.cast(
                inputs.itemById('createAsComponent')
            ).value
            ball_size = input_len_mm(inputs, 'ballSize')
            ball_clearance_ring = input_len_mm(inputs, 'ballClearanceRing')
            ball_clearance_cage = input_len_mm(inputs, 'ballClearanceCage')
            ball_count = adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById('ballCount')).value
            inner_diameter = input_len_mm(inputs, 'innerDiameter')
            wall_tb = input_len_mm(inputs, 'wallTB')
            wall_lr = input_len_mm(inputs, 'wallLR')
            create_balls = adsk.core.BoolValueCommandInput.cast(inputs.itemById('createBalls')).value
            close_cage_ratio = adsk.core.ValueCommandInput.cast(inputs.itemById('closeCageRatio')).value

            create_bearing_geometry(
                design,
                create_as_component,
                ball_size,
                ball_clearance_ring,
                ball_clearance_cage,
                ball_count,
                inner_diameter,
                wall_tb,
                wall_lr,
                create_balls,
                close_cage_ratio
            )
        except:
            ui.messageBox(f'Failed:\n{traceback.format_exc()}')


class BearingInputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            event_args = adsk.core.InputChangedEventArgs.cast(args)
            if event_args.input.id in {'ballSize', 'innerDiameter', 'wallTB', 'wallLR'}:
                update_display_values(event_args.inputs)
        except:
            ui.messageBox(f'Failed:\n{traceback.format_exc()}')


class BearingCommandDestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        adsk.terminate()


class BearingCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            command = adsk.core.Command.cast(args.command)
            inputs = command.commandInputs

            inputs.addBoolValueInput('createAsComponent', 'Create as component', True, '', True)
            inputs.addBoolValueInput('createBalls', 'Create balls', True, '', False)
            inputs.addValueInput('ballSize', 'Ball size', 'mm', adsk.core.ValueInput.createByString('4 mm'))
            inputs.addValueInput(
                'ballClearanceRing',
                'Ball clearance ring',
                'mm',
                adsk.core.ValueInput.createByString('0.05 mm')
            )
            inputs.addValueInput(
                'ballClearanceCage',
                'Ball clearance cage',
                'mm',
                adsk.core.ValueInput.createByString('0.1 mm')
            )
            inputs.addValueInput(
                'closeCageRatio',
                'Close cage ratio',
                '',
                adsk.core.ValueInput.createByReal(0.3333)
            )
            inputs.addIntegerSpinnerCommandInput('ballCount', 'Ball count', 1, 999, 1, 10)
            inputs.addValueInput(
                'innerDiameter',
                'Inner diameter',
                'mm',
                adsk.core.ValueInput.createByString('50 mm')
            )
            inputs.addValueInput(
                'wallTB',
                'Top/bottom wall thickness',
                'mm',
                adsk.core.ValueInput.createByString('1 mm')
            )
            inputs.addValueInput(
                'wallLR',
                'Left/right wall thickness',
                'mm',
                adsk.core.ValueInput.createByString('1 mm')
            )

            inputs.addTextBoxCommandInput('outerDiameterDisplay', 'Outer diameter (display)', '', 1, True)
            inputs.addTextBoxCommandInput('heightDisplay', 'Height (display)', '', 1, True)
            update_display_values(inputs)

            on_execute = BearingCommandExecuteHandler()
            command.execute.add(on_execute)
            handlers.append(on_execute)

            on_changed = BearingInputChangedHandler()
            command.inputChanged.add(on_changed)
            handlers.append(on_changed)

            on_destroy = BearingCommandDestroyHandler()
            command.destroy.add(on_destroy)
            handlers.append(on_destroy)
        except:
            ui.messageBox(f'Failed:\n{traceback.format_exc()}')


def run(_context: str):
    try:
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()

        cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC)
        on_created = BearingCommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        handlers.append(on_created)

        cmd_def.execute()
        adsk.autoTerminate(False)
    except:
        ui.messageBox(f'Failed:\n{traceback.format_exc()}')


def stop(_context: str):
    try:
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
    except:
        ui.messageBox(f'Failed:\n{traceback.format_exc()}')
