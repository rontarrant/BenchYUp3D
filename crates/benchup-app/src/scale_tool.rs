//! Scale factors come from the same world-space targets as Move.
use super::*;

impl BenchUpApp {
    // A tiny reference amplifies pointer rounding and snap changes. Measure
    // usable handles in pixels so sensitivity stays consistent at every zoom.
    const MIN_SCALE_REFERENCE_PIXELS: f64 = 24.;

    fn scale_reference(edit: &Modeling, camera: &Camera, size: DVec2) -> Option<DVec3> {
        let reference = edit.anchor - edit.center;
        let reference = edit
            .scale_constraint
            .map_or(reference, |lock| lock.project(reference));
        let start = camera.project(edit.anchor, size)?;
        let center = camera.project(edit.anchor - reference, size)?;
        (reference.length_squared() > 1e-12
            && start.distance(center) >= Self::MIN_SCALE_REFERENCE_PIXELS)
            .then_some(reference)
    }

    pub(super) fn scale_snap(
        edit: &Modeling,
        mut snap: Snap,
        grid_spacing: Option<f64>,
        snap_to_axes: bool,
    ) -> Snap {
        if let Some(spacing) = grid_spacing {
            if snap.kind == Inference::Plane
                && let Some(AxisConstraint::Axis(axis)) = edit.scale_constraint
            {
                snap.kind = Inference::Axis(axis);
            }
            grid::snap_with_geometry(
                snap,
                spacing,
                match edit.scale_constraint {
                    Some(AxisConstraint::Plane(normal)) => Some(normal.vector()),
                    _ => None,
                },
                grid::movement_axis(
                    snap.point,
                    Some(edit.anchor),
                    edit.scale_constraint,
                    snap_to_axes,
                ),
                &edit.base,
                Some(&edit.snap_excluded),
            )
        } else {
            snap
        }
    }

    pub(super) fn scale_value(
        edit: &Modeling,
        camera: &Camera,
        size: DVec2,
        snap_to_grid: bool,
    ) -> f64 {
        let project = |delta| {
            edit.scale_constraint
                .map_or(delta, |lock| lock.project(delta))
        };
        // A geometry endpoint is an exact world-space target.  Its scale
        // factor must not depend on how foreshortened the reference happens
        // to be on screen (the pixel-sized fallback is for free dragging).
        // In particular, Y-up camera angles can make a 3D reference look
        // shorter than MIN_SCALE_REFERENCE_PIXELS even at a normal zoom.
        // Close to the initial click, a few pixels of pointer motion must not
        // become a large scale jump merely because a nearby vertex/face snaps.
        // Preserve exact world-space geometry ratios once the drag is deliberate.
        // A foreshortened but nonzero reference can move only a few pixels
        // even for a substantial world-space scale (the parallel-view 1.5x
        // case moves about four pixels). Treat the drag as centre-near only
        // when the reference itself is nearly at the centre, or the pointer
        // displacement is small relative to that reference. This retains
        // stable three-pixel centre drags without discarding real scaling.
        let screen_reference = camera.project(edit.anchor, size)
            .zip(camera.project(edit.center, size))
            .map(|(anchor, center)| anchor.distance(center))
            .unwrap_or(0.);
        let pointer_motion = dv(edit.pointer - edit.pixel).length();
        let near_click = Self::scale_reference(edit, camera, size).is_none()
            && pointer_motion <= 6.
            && (screen_reference <= 6. || pointer_motion < screen_reference * 0.4);
        if !near_click && edit.scale_constraint.is_none()
            && edit.snap.is_some_and(|snap| matches!(snap.kind,
                Inference::Endpoint | Inference::Midpoint | Inference::Origin
                | Inference::FaceCenter | Inference::OnEdge | Inference::OnFace))
        {
            let reference = edit.anchor - edit.center;
            if reference.length_squared() > 1e-12 {
                return (reference + edit.direction).length() / reference.length();
            }
        }
        // Exact geometry must be resolved before the short-reference screen
        // fallback; otherwise the fallback overrides a snapped endpoint.
        if edit.scale_constraint.is_none()
            && let Some(value) = Self::unlocked_scale_value(edit, camera, size, snap_to_grid, near_click)
        {
            return value;
        }
        // A locked reference is defined in model coordinates, not by its
        // apparent screen length. Foreshortening must not change its ratio.
        // Retain the pixel fallback only for genuinely centre-near drags.
        let world_reference = project(edit.anchor - edit.center);
        let reference = Self::scale_reference(edit, camera, size).or_else(|| {
            (!near_click && edit.scale_constraint.is_some()
                && world_reference.length_squared() > 1e-12)
                .then_some(world_reference)
        });
        if let Some(reference) = reference {
            let target = reference + project(edit.direction);
            return if matches!(edit.scale_constraint, Some(AxisConstraint::Axis(_))) {
                target.dot(reference) / reference.length_squared()
            } else {
                target.length() / reference.length()
            };
        }

        // Center clicks, small selections, and foreshortened locked axes use
        // a visible selection extent instead of dividing by a tiny reference.
        // The snapped displacement still supplies the movement and direction.
        let extent = edit
            .base
            .selection_vertices(&edit.entities)
            .into_iter()
            .fold(DVec3::ZERO, |extent, id| {
                extent.max(project(edit.base.vertices[&id].position - edit.center).abs())
            });
        let Some(start) = camera.project(edit.anchor, size) else {
            return 1.;
        };
        let free = !snap_to_grid && edit.snap.is_some_and(|snap| {
            snap.kind == Inference::Plane
                || matches!(snap.kind, Inference::Axis(axis)
                    if edit.scale_constraint == Some(AxisConstraint::Axis(axis)))
        });
        let handle = [DVec3::X, DVec3::Y, DVec3::Z]
            .into_iter()
            .filter_map(|axis| {
                let world_handle = axis * extent.dot(axis);
                let endpoint = camera.project(edit.anchor + world_handle, size)?;
                let handle = endpoint - start;
                (handle.length_squared() > 1e-12).then_some((world_handle, handle))
            })
            .max_by(|(_, a), (_, b)| a.length_squared().total_cmp(&b.length_squared()));
        if let Some((world_handle, handle)) = handle {
            if near_click || free {
                // At a center click the world-space displacement can jump to
                // another snapped point, even for a three-pixel mouse move.
                // Use the actual mouse delta until the drag is deliberate;
                // this also keeps the initial scale at exactly 1x.
                1. + dv(edit.pointer - edit.pixel).dot(handle.normalize())
                    / handle.length().max(Self::MIN_SCALE_REFERENCE_PIXELS)
            } else {
                // Deliberate exact targets and grid steps retain world units.
                1. + project(edit.direction).dot(world_handle) / world_handle.length_squared()
            }
        } else {
            1.
        }
    }

    fn unlocked_scale_value(
        edit: &Modeling,
        camera: &Camera,
        size: DVec2,
        snap_to_grid: bool,
        near_click: bool,
    ) -> Option<f64> {
        let start = camera.project(edit.anchor, size)?;
        let center = camera.project(edit.center, size)?;
        let mut reference = start - center;
        // When no exact geometry or grid target is involved, the pointer is
        // authoritative. Inference may label the free point as an axis/face
        // rather than Plane, especially after the Y-up camera conversion.
        // Using the inferred 3D displacement then changes the apparent scale
        // of a parallel-projected drag (e.g. 1.5x becomes ~1.124x).
        let free_pointer = !snap_to_grid && !edit.snap.is_some_and(|snap| {
            matches!(snap.kind,
                Inference::Endpoint | Inference::Midpoint | Inference::Origin
                | Inference::FaceCenter | Inference::OnEdge)
        });
        if !free_pointer && reference.length() >= Self::MIN_SCALE_REFERENCE_PIXELS {
            // Exact geometry and grid targets retain world-space ratios.
            return None;
        }
        let movement = if near_click || free_pointer {
            dv(edit.pointer - edit.pixel)
        } else {
            camera.project(edit.anchor + edit.direction, size)? - start
        };
        if reference.length() < Self::MIN_SCALE_REFERENCE_PIXELS
            && (near_click || reference.length_squared() <= 1e-12)
        {
            // A center click has no radial direction. Use a stable screen-space
            // handle sized to the selection, without choosing a model axis.
            let radius = edit
                .base
                .selection_vertices(&edit.entities)
                .into_iter()
                .filter_map(|id| camera.project(edit.base.vertices[&id].position, size))
                .map(|point| point.distance(center))
                .fold(Self::MIN_SCALE_REFERENCE_PIXELS, f64::max);
            reference = DVec2::new(1., -1.).normalize() * radius;
        }
        // Distance responds to movement around the center in every direction;
        // projecting onto the original reference line would create an implicit lock.
        Some((reference + movement).length() / reference.length())
    }
}
