# Developer Cookbook
How to do common tasks within the Kalico.

## Moving the Toolhead
First get the toolhead object:
```
toolhead = self.printer.lookup_object("toolhead")
```
The toolhead object is not always available at module initalization time so it
should be looked up at time of use.

All moves with the toolhead are absolute moves. However you can instruct the
toolhead not to move an axis by passing `None` for a coordinate to
`manual_move()`. E.g this will move the toolhead to z=10mm at its current
x/y position:
```
toolhead.manual_move([None, None, 10.], speed)
```

To make a relative move, use the current toolhead position. This will lift
the Z axis by 10mm:
```
pos = toolhead.get_position()
toolhead.manual_move([None, None, pos[2] + 10.], speed)
```
