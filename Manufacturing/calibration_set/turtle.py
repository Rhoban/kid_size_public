import geometry as ge

class Turtle:
    def __init__(self, position=ge.Vec2d(0,0), direction=ge.Vec2d(1,0)):
        """
        Construct a turtle.

        >>> t = Turtle( ge.Vec2d(1, 2), ge.Vec2d(1, 0) )
        >>> t
        turtle(pos=[1, 2], dir=[1, 0])
        """
        self._position = position.clone()
        self._direction = direction.clone()
    def __repr__(self):
        """
        Return a string representation of the turtle.

        >>> t = Turtle( ge.Vec2d(1, 2), ge.Vec2d(1, 0) )
        >>> t
        turtle(pos=[1, 2], dir=[1, 0])
        """
        return "turtle(pos=%s, dir=%s)"%(
            str(self._position), str(self._direction)
        )
    def move_to(self, position):
        """
        Move the turtle to `position`.

        >>> t = Turtle( ge.Vec2d(1, 2), ge.Vec2d(1, 0) )
        >>> t.move_to( ge.Vec2d(3,4) )
        [3, 4]
        """
        self._position = position.clone()
        return self._position.clone()
    def forward(self, dist):
        """
        Forward the turtle from `dist`.

        >>> t = Turtle( ge.Vec2d(1, 2), ge.Vec2d(1, -1) )
        >>> t.forward( 3 )
        [4, -1]
        """
        self._position += (self._direction * dist)
        return self._position.clone()
    def left(self):
        """
        Turn the turtle to the left.

        >>> t = Turtle( ge.Vec2d(1, 2), ge.Vec2d(1, 0) )
        >>> t
        turtle(pos=[1, 2], dir=[1, 0])
        >>> t.left()
        >>> t
        turtle(pos=[1, 2], dir=[0, 1])
        >>> t.left()
        >>> t
        turtle(pos=[1, 2], dir=[-1, 0])
        """
        self._direction = self._direction.perpendicular()
    def right(self):
        """
        Turn the turtle to the right.

        >>> t = Turtle( ge.Vec2d(1, 2), ge.Vec2d(1, 0) )
        >>> t
        turtle(pos=[1, 2], dir=[1, 0])
        >>> t.right()
        >>> t
        turtle(pos=[1, 2], dir=[0, -1])
        >>> t.right()
        >>> t
        turtle(pos=[1, 2], dir=[-1, 0])
        """
        self._direction = - self._direction.perpendicular()
    def orient(self, direction):
        """
        Orient the turtle.

        >>> t = Turtle( ge.Vec2d(1, 2), ge.Vec2d(1, 0) )
        >>> t.orient( ge.Vec2d(0, 1) )
        >>> t
        turtle(pos=[1, 2], dir=[0, 1])
        """
        self._direction = direction.clone()

if __name__ == "__main__": 
    import doctest
    doctest.testmod()
