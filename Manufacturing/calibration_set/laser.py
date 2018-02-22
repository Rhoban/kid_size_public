import geometry
import math
import doctest


d = geometry.Drawing("Titre", "Description")

p1=[
    geometry.Vec2d(20,0), geometry.Vec2d(20,10), geometry.Vec2d(10,10), 
    geometry.Vec2d(10,20), geometry.Vec2d(20,20), geometry.Vec2d(20,30), 
    geometry.Vec2d(0,30), geometry.Vec2d(0,0)
]
p2=list( map(lambda x: x + geometry.Vec2d(40,0),p1) )
d.add(
    geometry.Closed_path(
        path=p1, offset=1,
        stroke_width=1, color="red", ref_color="black"
    )
)

d.add(
    geometry.Closed_path(
        path=p2, offset=-1,
        stroke_width=1, color="red", ref_color="black"
    )
)

d.add(
    geometry.Circle(
        center=geometry.Vec2d(90, 10), radius=10, 
        offset=1, stroke_width=1, color="red", ref_color="black"
    )
)

d.add(
    geometry.Circle(
        center=geometry.Vec2d(90, 10), radius=10, 
        offset=-1, stroke_width=1, color="red", ref_color="black"
    )
)
print( d.to_svg() )

