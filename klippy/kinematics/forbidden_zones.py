# Code for handling forbidden zones of movements
#
# Copyright (C) 2025  Daniel Berlin <dberlin@dberlin.org>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from shapely import from_wkt, STRtree, LineString
import logging

class ForbiddenZones:
    # On init we just read in the shapes and prepare the STRtree
    def __init__(self, config):
        self.forbidden_zone_tree = None
        if config.has_section("forbidden_zones"):
            section = config.getsection("forbidden_zones")
            forbidden_shapes_wkt = section.getlists("shapes", seps=("\n"))
            logging.info(f"Forbidden zone shapes (WKT):{forbidden_shapes_wkt}")
            forbidden_shapes_wkt = [shape.strip() for shape in forbidden_shapes_wkt if len(shape.strip()) != 0]
            forbidden_shapes = [from_wkt(shape) for shape in forbidden_shapes_wkt]
            logging.info(f"Forbidden zone shapes (parsed):{forbidden_shapes}")
            self.forbidden_zone_tree = STRtree(forbidden_shapes)
            
    # Check whether a proposed line move intersects any forbidden zone shapes
    # Touching the shape counts (So a point that is on the border of a shape is not allowed)
    def check_move(self, move):
        if self.forbidden_zone_tree:
            # Transform the move into a shapely LineString, get candidates from the STRtree
            # and check each candidate for intersection
            x_y_line = LineString([move.start_pos[:2], move.end_pos[:2]])
            candidates = self.forbidden_zone_tree.query(x_y_line, predicate='intersects')
            if len(candidates) > 0:
                for geom_index in candidates:
                    geom = self.forbidden_zone_tree.geometries.take(geom_index)
                    if x_y_line.intersects(geom):
                        raise move.move_error(f"Move line {x_y_line} intersects forbidden zone shape {geom}")
