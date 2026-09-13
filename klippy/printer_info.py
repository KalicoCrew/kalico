from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Callable

from .mathutil import Point

if TYPE_CHECKING:
    # To avoid circular imports/import issues, because . and .extras haven't been initialized yet,
    # these are only imported for type checking:
    from . import ConfigWrapper, Printer
    from .extras.probe import PrinterProbe

logger = logging.getLogger(__name__)


class PrinterInfo:
    bed_size: tuple[float, float] | None
    bed_corner_position: Point | None
    min_position: Point | None
    max_position: Point | None
    printer: Printer

    def __init__(self, config: ConfigWrapper) -> None:
        self.printer = config.get_printer()
        self.kinematics_name = config.get("kinematics")
        self.is_rectangular = self.kinematics_name in [
            "cartesian",
            "corexy",
            "corexz",
            "limited_cartesian",
            "limited_corexy",
            "limited_corexz",
            "deltesian",
        ]

        # This is hardcoded in bed_mesh.py in the BedMeshCalibrate._generate_points method, where
        #
        # x_dist = (max_x - min_x) / (x_cnt - 1)
        # y_dist = (max_y - min_y) / (y_cnt - 1)
        # # floor distances down to next hundredth
        # x_dist = math.floor(x_dist * 100) / 100
        # y_dist = math.floor(y_dist * 100) / 100
        # if x_dist < 1.0 or y_dist < 1.0:
        #     raise error("bed_mesh: min/max points too close together")
        #
        # The minimum probe count (x_cnt/y_cnt) is hardcoded to 3. With this
        # the minimum will be:
        #
        # dist / (*_cnt - 1) < 1.0
        # <=> dist < 1.0 * (*_cnt - 1)
        # <=> dist < 1.0 * (3 - 1)
        # <=> dist < 2.0
        #
        # TODO: Maybe refactor the code that enforces this into a shared function, so it is not hardcoded in multiple locations?
        self.min_mesh_size = Point(2.0, 2.0)
        self.bed_size = config.getfloatlist("bed_size", count=2, default=None)
        corner = config.getfloatlist(
            "bed_corner_position", count=2, default=None
        )
        self.bed_corner_position = (
            Point(*corner) if corner is not None else None
        )

        if not self.is_rectangular and (
            self.bed_size is not None or self.bed_corner_position is not None
        ):
            raise config.error(
                f"bed_size and bed_corner_position are not supported for"
                f" {self.kinematics_name} kinematics"
            )

        if self.bed_size is not None and (
            self.bed_size[0] <= 0 or self.bed_size[1] <= 0
        ):
            raise config.error(
                f"Invalid bed size {self.bed_size}, should be a positive value"
            )

        toolhead = self.printer.lookup_object("toolhead")
        curtime = self.printer.get_reactor().monotonic()

        kin_status = toolhead.get_kinematics().get_status(curtime)
        if "axis_minimum" in kin_status:
            self.min_position = Point(
                kin_status["axis_minimum"][0], kin_status["axis_minimum"][1]
            )
        else:
            self.min_position = None

        if "axis_maximum" in kin_status:
            self.max_position = Point(
                kin_status["axis_maximum"][0], kin_status["axis_maximum"][1]
            )
        else:
            self.max_position = None

    def nearest_point(self, point: Point) -> Point:
        """
        Ensures the given point is within the printers reachable area,
        returning the point itself or adjusting it to the nearest reachable point.

        This assumes both self.min_position and self.max_position are not None.

        """
        x, y = point

        x = max(min(x, self.max_position.x), self.min_position.x)
        y = max(min(y, self.max_position.y), self.min_position.y)

        return Point(x, y)

    def _probe_margin(self) -> Point:
        probe: PrinterProbe = self.printer.lookup_object("probe", None)
        if probe is None:
            return Point.origin()

        if probe.get_min_edge_distance() is None:
            return Point.origin()

        edge_distance = max(probe.get_min_edge_distance(), 0.0)
        return Point(edge_distance, edge_distance)

    def _mesh_min(self, probe_offset: Point) -> Point:
        # The bed corner position defines where the bed begins. Given that the probe
        # has to be a certain distance from the edge, the margin is added.
        # This results in the outermost point where the probe could be.
        #
        # For a better understanding, this is the bed, and point A is the bed corner position:
        #
        # +----------------+
        # |                |
        # |  +----------+  |
        # |  |          |  |
        # |  |          |  |
        # |  |          |  |
        # |  |          |  |
        # |  1----------+  |
        # |                |
        # A----------------+
        #
        # The inner square is the area where we can safely probe, and we are calculating the point 1.
        desired_probe_min = self.bed_corner_position + self._probe_margin()
        # This is the point where the probe could reach based on the physical limits of the printer:
        reachable_probe_min = self.min_position + probe_offset
        # The above points are small values like (5, 2) and (-5, -5). And the more the values increase,
        # the closer it gets to the center of the bed which could for example be at (150, 150).
        #
        # So by taking the max of these two points, we ensure that we can move there and if both points
        # are reachable, we take the one that is further from the edge to ensure we are within the margins.
        probe_min = Point(
            max(desired_probe_min.x, reachable_probe_min.x),
            max(desired_probe_min.y, reachable_probe_min.y),
        )

        # The above calculations were for the probe, but there is an offset between the probe and the nozzle.
        # The point should be reachable by the nozzle as well, which this ensures:
        return self.nearest_point(probe_min - probe_offset)

    def _mesh_max(self, probe_offset: Point) -> Point:
        # This is the same as _mesh_min, but for the other corner of the bed.
        bed_max = self.bed_corner_position + Point(*self.bed_size)

        desired_probe_max = bed_max - self._probe_margin()
        reachable_probe_max = self.max_position + probe_offset
        probe_max = Point(
            min(desired_probe_max.x, reachable_probe_max.x),
            min(desired_probe_max.y, reachable_probe_max.y),
        )

        return self.nearest_point(probe_max - probe_offset)

    def require_properties(
        self, properties: list[str], error: Callable[[str], Exception]
    ) -> None:
        missing_properties = [
            prop for prop in properties if getattr(self, prop) is None
        ]

        if len(missing_properties) > 0:
            raise error(
                f"the following options are required, but are not defined in the [printer] section: {', '.join(missing_properties)}"
            )

    def get_mesh_bounds(
        self,
        mesh_min: tuple[float, float] | None,
        mesh_max: tuple[float, float] | None,
        use_offsets: bool,
        error: Callable[[str], Exception],
        probe_offset: tuple[float, float] | None = None,
        target_aspect_ratio: float | None = None,
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        # If both are set, there is nothing to adjust:
        if mesh_min is not None and mesh_max is not None:
            return mesh_min, mesh_max

        if not self.is_rectangular:
            raise error(
                f"automatic mesh bounds calculation is not supported for"
                f" {self.kinematics_name} kinematics, please specify"
                f" mesh_min and mesh_max manually"
            )

        self.require_properties(
            ["bed_size", "bed_corner_position", "min_position", "max_position"],
            error,
        )

        can_move_min = mesh_min is None
        can_move_max = mesh_max is None

        if probe_offset is None:
            probe: PrinterProbe = self.printer.lookup_object("probe")
            offsets = probe.get_offsets()
            probe_offset = Point(offsets[0], offsets[1])
        else:
            probe_offset = Point(probe_offset[0], probe_offset[1])

        logger.debug(
            f"printer_info: mesh_min={mesh_min}, mesh_max={mesh_max},"
            f" probe_offset={probe_offset}, use_offsets={use_offsets},"
            f" target_aspect_ratio={target_aspect_ratio},"
            f" bed_corner_position={self.bed_corner_position},"
            f" bed_size={self.bed_size}, min_position={self.min_position},"
            f" max_position={self.max_position}, can_move_min={can_move_min},"
            f" can_move_max={can_move_max}"
        )

        # It is allowed to explicitly set one of the corners, and having it calculate the other
        # corner.
        #
        # This can be useful when you want to force how far the mesh should be from one of the edges.
        if mesh_min is None:
            mesh_min: Point = self._mesh_min(probe_offset)
        else:
            mesh_min: Point = Point(*mesh_min)
            if use_offsets:
                mesh_min -= probe_offset

        if mesh_max is None:
            mesh_max: Point = self._mesh_max(probe_offset)
        else:
            mesh_max: Point = Point(*mesh_max)
            if use_offsets:
                mesh_max -= probe_offset

        logger.debug(
            f"printer_info: calculated (nozzle coordinate) mesh_min={mesh_min}, mesh_max={mesh_max}"
        )

        mesh_delta = mesh_max - mesh_min
        if (
            mesh_delta.x < self.min_mesh_size.x
            or mesh_delta.y < self.min_mesh_size.y
        ):
            raise error(
                f"failed to calculate mesh_min and mesh_max, because the resulting mesh"
                f" (mesh_min={mesh_min}, mesh_max={mesh_max}) is too small, minimum size"
                f" {self.min_mesh_size}, but got {mesh_delta}.\n"
                "Please ensure the physical properties are correctly defined and the probe"
                " edge distance is not too large."
            )

        # In many cases the probe offset is only on one axis, e.g. only in the X direction, and 0 in the Y direction.
        # This can result in a mesh that has a different aspect ratio than the bed.
        #
        # This can be a problem with for example quad gantry level, where it expects the aspect ratio to match
        # the gantry geometry.
        #
        # Therefore it is allowed to specify a target aspect ratio, defined as x width divided by y width.
        # The mesh will be shrunk to fit the target aspect ratio. Given that the mesh is shrunk, it should not
        # be possible to reach outside of the printer's reachable area.
        if target_aspect_ratio is not None and (can_move_min or can_move_max):
            if target_aspect_ratio <= 0:
                raise error(
                    f"failed to adjust mesh to {target_aspect_ratio}, because it is not a positive value"
                )

            current_aspect_ratio = mesh_delta.x / mesh_delta.y

            # First we calculate how the mesh has to be adjusted to fit the target aspect ratio.
            #
            # In general there are two options to adjust the aspect ratio, by shrinking one dimension
            # or by increasing the other dimension.
            #
            # But we want to stay within the bounds, therefore only shinking is allowed:
            if current_aspect_ratio > target_aspect_ratio:
                # The mesh is wider than the target ratio, so we need to reduce the width or increase the height
                # but given that we want to be inside the margins, only reducing the width is an option:
                new_mesh_width = mesh_delta.y * target_aspect_ratio
                delta = Point(mesh_delta.x - new_mesh_width, 0)
            elif current_aspect_ratio < target_aspect_ratio:
                # The mesh is taller than the target ratio, so we need to reduce the height or increase the width
                # but given that we want to be inside the margins, only reducing the height is an option:
                new_mesh_height = mesh_delta.x / target_aspect_ratio
                delta = Point(0, mesh_delta.y - new_mesh_height)
            else:
                delta = Point(0, 0)

            # For the min we would have to add the delta, for the max we would have to subtract the delta,
            # ideally we would want to do half and half, but this is only possible if both mesh_min and mesh_max
            # are movable:
            if can_move_min and can_move_max:
                mesh_min += delta / 2
                mesh_max -= delta / 2
            elif can_move_min:
                mesh_min += delta
            elif can_move_max:
                mesh_max -= delta

        # After adjusting the mesh corners, it might have become too small, so this should be checked again:
        mesh_delta = mesh_max - mesh_min
        if (
            mesh_delta.x < self.min_mesh_size.x
            or mesh_delta.y < self.min_mesh_size.y
        ):
            raise error(
                f"failed to adjust mesh to match target aspect ratio {target_aspect_ratio}, because the resulting mesh"
                f" (mesh_min={mesh_min}, mesh_max={mesh_max}) is too small, minimum size is {self.min_mesh_size},"
                f" but got {mesh_delta}"
            )

        if use_offsets:
            mesh_min += probe_offset
            mesh_max += probe_offset

        logger.debug(
            f"printer_info: returning mesh_min={mesh_min}, mesh_max={mesh_max}"
            f" with use_offsets={use_offsets} and probe_offset={probe_offset}"
        )

        return ((mesh_min.x, mesh_min.y), (mesh_max.x, mesh_max.y))


def add_printer_objects(config: ConfigWrapper) -> None:
    config.get_printer().add_object("printer_info", PrinterInfo(config))
