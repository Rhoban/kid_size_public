import math

import sys
if sys.version_info[0] < 3:
    raise ValueError("Use Python 3 instead of Python %s."%(sys.version_info[0]))

class Vec2d:
    def __init__(self, x, y):
        """
        Construt a new 2D vector

        >>> v = Vec2d(3,4)
        >>> v
        [3, 4]
        """
        self.x = x
        self.y = y

    def __eq__(self, vec):
        """
        Construt a new 2D vector

        >>> v1 = Vec2d(3,4)
        >>> v2 = Vec2d(1,4)
        >>> v3 = Vec2d(3,1)
        >>> v4 = Vec2d(3,4)
        >>> v1 == v2
        False
        >>> v1 == v3
        False
        >>> v1 == v4
        True
        >>> v1 is v4
        False
        """
        return self.x == vec.x and self.y == vec.y

    def clone(self):
        """
        Clone a vector

        >>> v = Vec2d(3,4)
        >>> v is v.clone()
        False
        >>> v == v.clone()
        True
        """
        return Vec2d( self.x, self.y )

    def __repr__(self):
        """
        Return the string representation of a vector

        >>> v = Vec2d(5,1)
        >>> v
        [5, 1]
        """
        return "[" + str(self.x) + ", " + str(self.y) + "]"


    def __add__( self, vec ):
        """
        Addition of two vectors.

        >>> v1 = Vec2d(1,2)
        >>> v2 = Vec2d(3,4)
        >>> v1 + v2
        [4, 6]
        """
        return Vec2d( self.x + vec.x, self.y + vec.y )

    def __mul__(self, alpha):
        """
        Return the multiplication of the vector 'self' with a scalar 'alpha'.

        >>> v1 = Vec2d(1,2)
        >>> v1 * 3
        [3, 6]
        """
        return Vec2d( self.x * alpha, self.y * alpha )

    def __truediv__(self, alpha):
        """
        Return the divition of the vector 'self' by a scalar 'alpha'.

        >>> v1 = Vec2d(3,6)
        >>> v1 / 3
        [1.0, 2.0]
        """
        return Vec2d( self.x / alpha, self.y / alpha )

    def __iadd__( self, vec ):
        """
        Additive operation on `self`.

        >>> v1 = Vec2d(1,2)
        >>> v1 += Vec2d(3,4)
        >>> v1
        [4, 6]
        """
        self.x += vec.x
        self.y += vec.y
        return self

    def __neg__(self):
        """
        Return the opposite vector.

        >>> v1 = Vec2d(3,4)
        >>> -v1
        [-3, -4]
        """
        return Vec2d( -self.x, -self.y )

    def __sub__(self, vec):
        """
        Minus operator?

        >>> v1 = Vec2d(3,4)
        >>> v2 = Vec2d(1,3)
        >>> v1 - v2
        [2, 1]
        """
        return Vec2d( self.x - vec.x, self.y - vec.y )

    def norm2(self ):
        """
        Return the norm power two of the vector.

        >>> v1 = Vec2d(3,4)
        >>> v1.norm2()
        25
        """
        return self.x*self.x + self.y*self.y

    def norm(self):
        """
        Return the norm of the vector.

        >>> v1 = Vec2d(3,4)
        >>> v1.norm()
        5.0
        """
        return math.sqrt( self.norm2() )
    def normalize(self):
        """
        Normalize the vector

        >>> v1 = Vec2d(3,4)
        >>> v1.normalize()
        >>> v1
        [0.6, 0.8]
        """
        n = self.norm()
        self.x = self.x/n
        self.y = self.y/n
    def rotate( self, angle ):
        """
        Rotate the vector by an angle of `angle` radians.

        >>> import math
        >>> v1 = Vec2d(1,0)
        >>> v2 = v1.rotate( math.pi/2.0)
        >>> ( v2 - Vec2d(0,1) ).norm() < 0.001
        True
        >>> v1 = Vec2d(0,1)
        >>> v2 = v1.rotate( math.pi/2.0)
        >>> ( v2 - Vec2d(-1,0) ).norm() < 0.001
        True
        """
        return Vec2d(
            self.x * math.cos(angle) - self.y * math.sin(angle),
            self.x * math.sin(angle) + self.y * math.cos(angle)
        )
    def perpendicular(self):
        """
        Return the vector perpendicular (turn anti-clockwise).

        >>> v = Vec2d(1, 2)
        >>> v.perpendicular()
        [-2, 1]
        """
        return Vec2d( -self.y, self.x)
    def scalar_product( self, vec ):
        """
        Return the scalar product of `self`with `vec`.

        >>> v1 = Vec2d(2,3)
        >>> v2 = Vec2d(5,6)
        >>> v1.scalar_product(v2)
        28
        """
        return self.x * vec.x + self.y * vec.y

    def det( self, vec ):
        """
        Compute the determinant of the matrix where the first vector is
        `self` and the second vector is `vec`.
        >>> v1 = Vec2d(2,3)
        >>> v2 = Vec2d(3,4)
        >>> v1.det( v2 )
        -1
        """
        return self.x * vec.y - self.y * vec.x

    def vectorial_product( self, vec):
        """
        Compute the vectorial product of self with vec

        >>> v1 = Vec2d(2,3)
        >>> v2 = Vec2d(3,4)
        >>> v1.vectorial_product( v2 )
        -1
        """
        return self.det(vec)

class Matrix2d:
    def __init__(self, a, b, c, d):
        """
        matrix 2X2
           [ a b ]
           [ c d ]
        >>> M = Matrix2d( 1,2,3,5 )
        >>> M
        [[1, 2], [3, 5]]
        """
        self._a = a
        self._b = b
        self._c = c
        self._d = d

    def det(self):
        """
        Determinant of the matrix

        >>> M = Matrix2d( 1,2,3,5 )
        >>> M.det()
        -1
        >>> M = Matrix2d( 1,2,2,4 )
        >>> M.det()
        0
        >>> M = Matrix2d( 1,0,0,1 )
        >>> M.det()
        1
        >>> M = Matrix2d( 0,1,1,0 )
        >>> M.det()
        -1
        """
        return self._a * self._d - self._c * self._b

    def __repr__(self):
        """
        >>> M = Matrix2d( 1,2,3,5 )
        >>> M
        [[1, 2], [3, 5]]
        """
        return str(
            [[ self._a, self._b], [self._c, self._d]]
        )

    def __mul__(self, elem):
        """
        Matricial product

        >>> M = Matrix2d( 1,2,3,4 )
        >>> M*2
        [[2, 4], [6, 8]]
        >>> M*Vec2d(2, 0)
        [2, 6]
        >>> M*Vec2d(0, 2)
        [4, 8]
        >>> M*Vec2d(0, 0)
        [0, 0]
        >>> M*Matrix2d(2, 0, 0, 0)
        [[2, 0], [6, 0]]
        >>> M*Matrix2d(0, 2, 0, 0)
        [[0, 2], [0, 6]]
        >>> M*Matrix2d(0, 0, 2, 0)
        [[4, 0], [8, 0]]
        >>> M*Matrix2d(0, 0, 0, 2)
        [[0, 4], [0, 8]]
        >>> M*Matrix2d(0, 0, 0, 0)
        [[0, 0], [0, 0]]
        """
        if type(elem) is int or type(elem) is float:
            return Matrix2d(
                self._a * elem, self._b * elem,
                self._c * elem, self._d * elem
            )
        if type(elem) is Vec2d:
            return Vec2d(
                self._a * elem.x + self._b * elem.y,
                self._c * elem.x + self._d * elem.y,
            )
        if type(elem) is Matrix2d:
            return Matrix2d(
                self._a * elem._a + self._b * elem._c,
                self._a * elem._b + self._b * elem._d,
                self._c * elem._a + self._d * elem._c,
                self._c * elem._b + self._d * elem._d,
            )

    def inverse(self):
        """
        The inverse of the matrix
        >>> M = Matrix2d( 2,4,3,5 )
        >>> M.inverse()
        [[-2.5, 2.0], [1.5, -1.0]]
        >>> M * M.inverse()
        [[1.0, 0.0], [0.0, 1.0]]
        >>> M.inverse() * M
        [[1.0, 0.0], [0.0, 1.0]]
        """
        return Matrix2d( self._d, - self._b, - self._c, self._a ) * (1/self.det())

class Line:
    def __init__(self, point1, point2):
        """
        Construct a line

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> l = Line( point1, point2 )
        >>> l
        l([1, 2], [3, 4])
        """
        self._point1 = point1.clone()
        self._point2 = point2.clone()

    def _half_space(self, point):
        """
        Return a real closed to 0 if the point is on the line, 
        a negative real if it is on the right and a positive real if it is on 
        the left.
        """
        return (self._point2 - self._point1).det(point - self._point1)

    def is_inside(self, point, error=0.00000001):
        """
        Return True if the point is closed to the line.

        >>> Line(Vec2d(0,2), Vec2d(4,4)).is_inside(Vec2d(2,3), error=0.01)
        True
        >>> Line(Vec2d(0,2), Vec2d(4,5)).is_inside(Vec2d(2,3), error=0.01)
        False
        """
        return abs( self._half_space(point) ) < error

    def is_on_the_left( self, point ):
        """
        Return True if the point `point` is on the left of the line.
        The line is directed from self._point1 to self._point2.

        >>> Line(Vec2d(0,2), Vec2d(4,5)).is_on_the_left(Vec2d(2,3))
        False
        >>> Line(Vec2d(0,2), Vec2d(4,5)).is_on_the_left(Vec2d(2,5))
        True
        """
        return self._half_space(point) > 0

    def is_on_the_right( self, point ):
        """
        Return True if the point `point` is on the right of the line.
        The line is directed from self._point1 to self._point2.

        >>> Line(Vec2d(0,2), Vec2d(4,5)).is_on_the_right(Vec2d(2,3))
        True
        >>> Line(Vec2d(0,2), Vec2d(4,5)).is_on_the_right(Vec2d(2,5))
        False
        """
        return self._half_space(point) < 0

    def __repr__(self):
        """
        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> l = Line( point1, point2 )
        >>> l
        l([1, 2], [3, 4])
        """
        return "l(%s, %s)"%(str(self._point1), str(self._point2))

    def intersection( self, line ):
        """
        Compute the intersection between `self` and `line`.

        >>> l1 = Line( Vec2d(0,0), Vec2d(1,1) )
        >>> l2 = Line( Vec2d(1,0), Vec2d(0,1) )
        >>> l1.intersection(l2)
        [[0.5, 0.5]]
        >>> l1 = Line( Vec2d(0,2), Vec2d(4,4) )
        >>> l2 = Line( Vec2d(1,0), Vec2d(3,6) )
        >>> i = l1.intersection(l2) 
        >>> ( i[0] - Vec2d(2.0, 3.0) ).norm() < 0.000000001
        True
        """
        [a1, b1, c1] = self.equation()
        [a2, b2, c2] = line.equation()
        m = Matrix2d(a1, b1, a2, b2)
        if abs(m.det()) < 0.000001 :
            return []
        return [m.inverse() * Vec2d(-c1, -c2)]

    def equation(self):
        """
        Return the coefficitents [a, b, c] of the equation a.x + b.y + c = 0
        of the line.

        >>> l = Line( Vec2d(0,0), Vec2d(1,0) )
        >>> l.equation()
        [0, -1, 0]
        >>> l = Line( Vec2d(0,0), Vec2d(0,1) )
        >>> l.equation()
        [1, 0, 0]
        >>> l = Line( Vec2d(1,2), Vec2d(3,5) )
        >>> l.equation()
        [3, -2, 1]
        """
        direction = self._point2 - self._point1 
        a = Vec2d(1,0).det( direction )
        b = Vec2d(0,1).det( direction )
        c = - self._point1.det( direction )
        return [a, b, c]


class Graphic:
    def __init__(self):
        """
            Construct a Graphic component.

            >>> class A(Graphic):
            ...     def __init__(self):
            ...         Graphic.__init__(self)
            ...     def geometry(self):
            ...         return [1,2,3,5]
            >>> a = A()
        """
        pass
    def width(self):
        """
            Return the width of the graphic component.

            >>> class A(Graphic):
            ...     def __init__(self):
            ...         Graphic.__init__(self)
            ...     def geometry(self):
            ...         return [1,2,3,5]
            >>> a = A()
            >>> a.width()
            2
        """
        [mi_x, mi_y, ma_x, ma_y] = self.geometry()
        return ma_x - mi_x 

    def height(self):
        """
            Return the height of the graphic component.

            >>> class A(Graphic):
            ...     def __init__(self):
            ...         Graphic.__init__(self)
            ...     def geometry(self):
            ...         return [1,2,3,5]
            >>> a = A()
            >>> a.height()
            3
        """
        [mi_x, mi_y, ma_x, ma_y] = self.geometry()
        return ma_y - mi_y 

class Segment(Graphic):
    def __init__(self, point1, point2):
        """
        Construct a segment

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> s = Segment( point1, point2 )
        >>> s
        s([1, 2], [3, 4])
        """
        Graphic.__init__(self)
        self._point1 = point1.clone()
        self._point2 = point2.clone()
    def clone(self):
        """
        Clone a segment

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> s = Segment( point1, point2 )
        >>> s.clone() == s
        True
        >>> s.clone() is s
        False
        """
        return Segment( self._point1.clone(), self._point2.clone() )
    def __eq__(self, segment):
        """
        Test equality of segments

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> s1 = Segment( point1, point2 )
        >>> s2 = Segment( point1, point1 )
        >>> s3 = Segment( point2, point2 )
        >>> s4 = Segment( point1, point2 )
        >>> s1 == s2
        False
        >>> s1 == s3
        False
        >>> s1 == s4
        True
        """
        return self._point1 == segment._point1 and self._point2 == segment._point2

    def translate(self, vec):
        """
        Translate the segment

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> s = Segment( point1, point2 )
        >>> s.translate( Vec2d(2,3) )
        >>> s
        s([3, 5], [5, 7])
        """
        self._point1 += vec
        self._point2 += vec
    def origin(self):
        """
        Gives the origin of the segment.

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> s = Segment( point1, point2 )
        >>> s.origin()
        [1, 2]
        """
        return self._point1.clone()
    def end(self):
        """
        Gives the end of the segment.

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> s = Segment( point1, point2 )
        >>> s.end()
        [3, 4]
        """
        return self._point2.clone()
    def direction(self):
        """
        Returns the direction of the segment.

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 5)
        >>> s = Segment( point1, point2 )
        >>> s.direction()
        [2, 3]
        """
        return self._point2 - self._point1
    def normalized_direction(self):
        """
        Returns the direction of the segment.

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(4, 6)
        >>> s = Segment( point1, point2 )
        >>> s.normalized_direction()
        [0.6, 0.8]
        """
        v = self.direction()
        v.normalize()
        return v
    def normal(self):
        """
        Return the normal vector of the segment. The vector is oriented on the
        left.

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 5)
        >>> s = Segment( point1, point2 )
        >>> abs(s.normal().norm() - 1.0) < 0.0001
        True
        >>> abs( s.direction().scalar_product(s.normal()) ) < 0.0001
        True
        """
        v = self._point2 - self._point1
        n = Vec2d( -v.y, v.x )
        return n / n.norm()
    def shift(self, offset):
        """
        Shift the vector by an offset in the direction given by the normal
        of the segment.

        >>> point1 = Vec2d(0, 0)
        >>> point2 = Vec2d(0, 1)
        >>> s = Segment( point1, point2 )
        >>> s.shift(2)
        >>> s
        s([-2.0, 0.0], [-2.0, 1.0])
        >>> point1 = Vec2d(0, 0)
        >>> point2 = Vec2d(1, 0)
        >>> s = Segment( point1, point2 )
        >>> s.shift(2)
        >>> s
        s([0.0, 2.0], [1.0, 2.0])
        """
        d = self.direction()
        normal = Vec2d( -d.y, d.x )
        normal.normalize()
        self.translate( normal*offset )
    def shifted(self, offset):
        """
        Return a shift a vector

        >>> point1 = Vec2d(0, 0)
        >>> point2 = Vec2d(0, 1)
        >>> s = Segment( point1, point2 )
        >>> s.shifted(2)
        s([-2.0, 0.0], [-2.0, 1.0])
        >>> point1 = Vec2d(0, 0)
        >>> point2 = Vec2d(1, 0)
        >>> s = Segment( point1, point2 )
        >>> s.shifted(2)
        s([0.0, 2.0], [1.0, 2.0])
        """
        s = self.clone()
        s.shift(offset)
        return s
    def  __repr__(self):
        """
        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> s = Segment( point1, point2 )
        >>> s
        s([1, 2], [3, 4])
        """
        return "s(%s, %s)"%(str(self._point1), str(self._point2))
    def line(self):
        """
        Return the line of a segment.

        >>> point1 = Vec2d(1, 2)
        >>> point2 = Vec2d(3, 4)
        >>> s = Segment( point1, point2 )
        >>> s
        s([1, 2], [3, 4])
        >>> s.line()
        l([1, 2], [3, 4])
        """
        return Line(self._point1, self._point2)
    def intersection(self, elem):
        """
        Compute the intersection between `self` and line or a segment.

        >>> s1 = Line( Vec2d(0,0), Vec2d(1,1) )
        >>> s2 = Line( Vec2d(1,0), Vec2d(0,1) )
        >>> s1.intersection(s2)
        [[0.5, 0.5]]
        >>> s1 = Segment( Vec2d(0,2), Vec2d(4,4) )
        >>> s2 = Segment( Vec2d(1,0), Vec2d(3,6) )
        >>> i = s1.intersection(s2)
        >>> ( i[0] - Vec2d(2.0, 3.0) ).norm() < 0.000000001
        True
        >>> s1 = Segment( Vec2d(0,2), Vec2d(1,2.5) )
        >>> s2 = Segment( Vec2d(1,0), Vec2d(3,6) )
        >>> s1.intersection(s2)
        []
        >>> len(s1.line().intersection(s2.line()))
        1
        >>> s1 = Segment( Vec2d(0,2), Vec2d(4,4) )
        >>> s2 = Segment( Vec2d(1,0), Vec2d(1.5,1.5) )
        >>> s1.intersection(s2)
        []
        >>> len(s1.line().intersection(s2.line()))
        1
        """
        if type(elem) is Line:
            line = elem
        elif type(elem) is Segment:
            line = elem.line()
        else:
            raise ValueError("elem is not a Line or a Segment.")
        i = self.line().intersection( line )
        def is_in_segment( seg, point ):
            s1 = (point - seg._point1).scalar_product( seg._point2 - seg._point1 )
            s2 = (point - seg._point2).scalar_product( seg._point1 - seg._point2 )
            return s1 >=0 and s2 >=0
        if len(i) == 1 and is_in_segment( self, i[0] ) and ( 
            type(elem) is Line or is_in_segment( elem, i[0] )
        ):
            return i
        return []
    def to_svg(self, color, stroke_width=2):
        '''
        Return the svg code of a segment.

        >>> s = Segment( Vec2d(1,2), Vec2d(3,4) )
        >>> s.to_svg( 'red' )
        '<line x1="1.0" y1="2.0" x2="3.0" y2="4.0" stroke="red" stroke-width="2" />'
        '''
        return "<line x1=\"%s\" y1=\"%s\" x2=\"%s\" y2=\"%s\" stroke=\"%s\" stroke-width=\"%s\" />"%(
            str(float(self._point1.x)), str(float(self._point1.y)),
            str(float(self._point2.x)), str(float(self._point2.y)),
            color, int(stroke_width)
        )


class Drawing(Graphic):
    def __init__(self, title, description, unit='mm'):
        """
            Construct a new drawing.

            >>> d = Drawing("Titre", "Description")
            >>> p1=[Vec2d(20,0), Vec2d(20,10), Vec2d(10,10), Vec2d(10,20), Vec2d(20,20), Vec2d(20,30), Vec2d(0,30), Vec2d(0,0)]
            >>> d.add(
            ...    Closed_path(
            ...        path=p1, offset=0.15,
            ...        stroke_width=1, color="red", ref_color=None
            ...    )
            ... )
        """
        Graphic.__init__(self)
        self._shapes = []
        self._title = title
        self._description = description
        self._unit = unit

    def add(self, elem):
        """
            Construct a new drawing.

            >>> d = Drawing("Figure", "a polygon")
            >>> p1=[Vec2d(20,0), Vec2d(20,10), Vec2d(10,10), Vec2d(10,20), Vec2d(20,20), Vec2d(20,30), Vec2d(0,30), Vec2d(0,0)]
            >>> d.add(
            ...    Closed_path(
            ...        path=p1, offset=0.15,
            ...        stroke_width=1, color="red", ref_color=None
            ...    )
            ... )
            >>> d
            drawing("Figure","a polygon")
        """
        if type(elem) is list:
            for shape in elem:
                self.add(shape)
        elif type(elem) in [ Circle, Closed_path ]:
            self._shapes.append(elem)
        else:
            raise ValueError("Invalid element is given. It type (%s) is unknown."%(type(elem)))

    def __repr__(self):
        """
            Return a string representation of `self`.

            >>> d = Drawing("Figure", "a polygon")
            >>> d
            drawing("Figure","a polygon")
        """
        return "drawing(\"%s\",\"%s\")"%(self._title, self._description)

    def geometry(self):
        """
            Construct a new drawing.

            >>> d = Drawing("Titre", "Description")
            >>> p1=[Vec2d(20,0), Vec2d(20,10), Vec2d(10,10), Vec2d(10,20), Vec2d(20,20), Vec2d(20,30), Vec2d(0,30), Vec2d(0,0)]
            >>> d.add(
            ...    Closed_path(
            ...        path=p1, offset=0.15,
            ...        stroke_width=1, color="red", ref_color=None
            ...    )
            ... )
            >>> d.geometry()
            [-0.15, -0.15, 20.15, 30.15]
        """
        if( len(self._shapes) == 0 ):
            min_x, min_y, max_x, max_y = None, None, None, None
        else:
            min_x, min_y, max_x, max_y = self._shapes[0].geometry()

        for shape in self._shapes:
            g_min_x, g_min_y, g_max_x, g_max_y = shape.geometry()
            min_x = min( min_x, g_min_x )
            min_y = min( min_y, g_min_y )
            max_x = max( max_x, g_max_x )
            max_y = max( max_y, g_max_y )
        return [min_x, min_y, max_x, max_y]

    def to_svg(self):
        """
        Return the svg code of the drawing

        >>> d = Drawing("Figure", "Description")
        >>> p1 = [Vec2d(20,0), Vec2d(20,10), Vec2d(10,10)]
        >>> d.add(
        ...    Closed_path(
        ...        path=p1, offset=0.15,
        ...        stroke_width=1, color="red", ref_color="black"
        ...    )
        ... )
        >>> print(d.to_svg())
        <?xml version="1.0" encoding="utf-8"?>
        <svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="10mm" height="10mm" viewBox="9 0 20 10">
            <title>Figure</title>
            <desc>Description</desc>
            <path d="M 19.850000 0.362132 L 19.850000 9.850000 L 10.362132 9.850000 L 19.850000 0.362132" stroke="red" stroke-width="1" fill="none"  stroke-linecap="round" />
            <path d="M 20.000000 0.000000 L 20.000000 10.000000 L 10.000000 10.000000 L 20.000000 0.000000" stroke="black" stroke-width="1" fill="none" stroke-linecap="round" />
        </svg>
        """
        [mi_x, mi_y, ma_x, ma_y] = self.geometry()
        result = "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
        result += "\n<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"%d%s\" height=\"%d%s\" viewBox=\"%s %s %s %s\">"%(
            self.width(), self._unit, self.height(), self._unit,
            str(int(mi_x)), str(int(mi_y)), str(int(ma_x)), str(int(ma_y)) 
        )
        result += "\n    <title>%s</title>"%(self._title)
        result += "\n    <desc>%s</desc>"%(self._description)
        for shape in self._shapes:
            svg = "\n" + shape.to_svg()
            result += svg.replace('\n', '\n    ')
        result += "\n</svg>"
        return result

    def draw(self, file_path):
        """
        First we generate a temporary file path

        >>> import tempfile
        >>> with tempfile.NamedTemporaryFile(dir='/tmp', delete=True) as tmpfile:
        ...     temp_file_name = tmpfile.name

        Now we generate the svg image
        >>> d = Drawing("Figure", "Description")
        >>> p1 = [Vec2d(20,0), Vec2d(20,10), Vec2d(10,10)]
        >>> d.add(
        ...    Closed_path(
        ...        path=p1, offset=0.15,
        ...        stroke_width=1, color="red", ref_color=None
        ...    )
        ... )
        >>> d.draw(temp_file_name + '.svg')
        """
        import codecs
        f = codecs.open(file_path, "w", "utf-8")
        f.write(self.to_svg())
        f.close()

class Circle(Graphic):
    def __init__(self, center, radius, offset, stroke_width=2, color="red", ref_color=None, fill_color=None ):
        """
        Construct a circle.

        >>> c = Circle(center=Vec2d(2,3), radius=10, offset=1)
        >>> c
        c([2, 3], 10, off=1)
        """
        Graphic.__init__(self)
        self._radius = radius
        if not type(center) is Vec2d:
            raise ValueError("Invalid type of center. We expect a " + str(type(Vec2d)) + " type.")
        self._center = center
        self._offset = offset
        self._color = color
        self._ref_color = ref_color
        if fill_color is None:
            self._fill_color = "none"
        else:
            self._fill_color = fill_color
        self._stroke_width = stroke_width
    def __repr__(self):
        """
        Return a string representation of  circle.

        >>> c = Circle(center=Vec2d(2,3), radius=10, offset=1)
        >>> c
        c([2, 3], 10, off=1)
        """
        return "c(%s, %s, off=%s)"%(
            str(self._center), str(self._radius), str(self._offset)
        )
    def geometry(self):
        """
        Return the geometry of a circle.

        >>> c = Circle(center=Vec2d(2,3), radius=10, offset=1)
        >>> c.geometry()
        [-7, -6, 11, 12]
        >>> c = Circle(center=Vec2d(2,3), radius=10, offset=-1)
        >>> c.geometry()
        [-9, -8, 13, 14]
        """
        offset_radius = self._radius - self._offset
        return [
            self._center.x - offset_radius, self._center.y - offset_radius,
            self._center.x + offset_radius, self._center.y + offset_radius
        ]
    def to_svg(self, stroke_width=None, color=None, ref_color=None, fill_color=None):
        """
        Return the svg code of a circle.

        >>> c = Circle(center=Vec2d(2,3), radius=10, offset=1)
        >>> print( c.to_svg() )
        <circle cx="2.000000" cy="3.000000" r="9.000000" stroke="red" stroke-width="2" fill="none" stroke-linecap="round" />
        >>> print( c.to_svg(ref_color="black") )
        <circle cx="2.000000" cy="3.000000" r="9.000000" stroke="red" stroke-width="2" fill="none" stroke-linecap="round" />
        <circle cx="2.000000" cy="3.000000" r="10.000000" stroke="black" stroke-width="2" fill="none" stroke-linecap="round" />
        """
        result = ""
        if stroke_width is None:
            stroke_width = self._stroke_width
        if color is None:
            color = self._color
        if ref_color is None:
            ref_color = self._ref_color
        if fill_color is None:
            fill_color = self._fill_color
        offset_radius = self._radius - self._offset
        result += "<circle cx=\"%f\" cy=\"%f\" r=\"%f\" stroke=\"%s\" stroke-width=\"%d\" fill=\"%s\" stroke-linecap=\"round\" />"%(
            self._center.x, self._center.y, offset_radius, color, stroke_width, fill_color
        )
        if not ref_color is None:
            result += "\n<circle cx=\"%f\" cy=\"%f\" r=\"%f\" stroke=\"%s\" stroke-width=\"%d\" fill=\"%s\" stroke-linecap=\"round\" />"%(
                self._center.x, self._center.y, self._radius, ref_color, stroke_width, fill_color
            )
        return result

class Closed_path(Graphic):
    def __init__(self, path=None, offset=0, stroke_width=2, color="red", ref_color=None ):
        """
        >>> p = Closed_path( [Vec2d(1,3), Vec2d(2,5), Vec2d(4,4)] )
        >>> p
        cp([[1, 3], [2, 5], [4, 4]], off=0)
        """
        Graphic.__init__(self)
        if path is None:
            self._path = []
        else:
            self._path = path
        self._offset = offset
        self._color = color
        self._ref_color = ref_color
        self._stroke_width = stroke_width
    def __iadd__( self, elem ):
        """
        Add a point in the path.

        >>> p = Closed_path( [Vec2d(1,3), Vec2d(2,5), Vec2d(4,4)] )
        >>> p += Vec2d(8,3)
        >>> p
        cp([[1, 3], [2, 5], [4, 4], [8, 3]], off=0)
        """
        if type(elem) is Vec2d:
            point_list = [elem]
        elif type(elem) is List:
            point_list = elem
        else:
            raise ValueError("A point or a list od point is needed.")
        for point in point_list:
            self._path.append(point)
        return self
    def geometry(self):
        """
        >>> p = Closed_path( path=[Vec2d(1,3), Vec2d(2,5), Vec2d(4,4)], offset=-0.1 )
        >>> p.geometry()
        [0.9, 2.9, 4.1, 5.1]
        """
        point = self._path[0]
        min_x = point.x
        min_y = point.y
        max_x = point.x
        max_y = point.y
        for point in self._path:
            x = point.x
            y = point.y
            min_x = min( min_x, x )
            min_y = min( min_y, y )
            max_x = max( max_x, x )
            max_y = max( max_y, y )
        return [min_x-abs(self._offset), min_y-abs(self._offset), max_x+abs(self._offset), max_y+abs(self._offset)]
    def __repr__(self):
        """
        >>> p = Closed_path( path=[Vec2d(1,3), Vec2d(2,5), Vec2d(4,4)], offset=-0.1 )
        >>> p
        cp([[1, 3], [2, 5], [4, 4]], off=-0.1)
        """
        return "cp(%s, off=%s)"%(str(self._path), str(self._offset))
    def to_svg(self, stroke_width=None, color=None, ref_color=None):
        """
        Return the svg code of the closed path.
        >>> p1 = [Vec2d(20,0), Vec2d(20,10), Vec2d(10,10)]
        >>> cl = Closed_path(
        ...     path=p1, offset=0.15,
        ...     stroke_width=1, color="red", ref_color="black"
        ... )
        >>> print(cl.to_svg())
        <path d="M 19.850000 0.362132 L 19.850000 9.850000 L 10.362132 9.850000 L 19.850000 0.362132" stroke="red" stroke-width="1" fill="none"  stroke-linecap="round" />
        <path d="M 20.000000 0.000000 L 20.000000 10.000000 L 10.000000 10.000000 L 20.000000 0.000000" stroke="black" stroke-width="1" fill="none" stroke-linecap="round" />


        >>> p = [
        ...    Vec2d(0,0), Vec2d(200,0), Vec2d(200,100), Vec2d(100,100),
        ...    Vec2d(100,200), Vec2d(200,200), Vec2d(200,300), Vec2d(0,300)
        ... ]
        >>> cl1 = Closed_path(
        ...     path=p, offset=-10,
        ...     stroke_width=1, color="red", ref_color="black"
        ... )
        >>> print(cl1.to_svg())
        <path d="M 10.000000 -10.000000 L 190.000000 -10.000000 A 20.000000 20.000000 0.000000 0 1 210.000000 10.000000 L 210.000000 90.000000 A 20.000000 20.000000 0.000000 0 1 190.000000 110.000000 L 110.000000 110.000000 L 110.000000 190.000000 L 190.000000 190.000000 A 20.000000 20.000000 0.000000 0 1 210.000000 210.000000 L 210.000000 290.000000 A 20.000000 20.000000 0.000000 0 1 190.000000 310.000000 L 10.000000 310.000000 A 20.000000 20.000000 0.000000 0 1 -10.000000 290.000000 L -10.000000 10.000000 A 20.000000 20.000000 0.000000 0 1 10.000000 -10.000000" stroke="red" stroke-width="1" fill="none"  stroke-linecap="round" />
        <path d="M 0.000000 0.000000 L 200.000000 0.000000 L 200.000000 100.000000 L 100.000000 100.000000 L 100.000000 200.000000 L 200.000000 200.000000 L 200.000000 300.000000 L 0.000000 300.000000 L 0.000000 0.000000" stroke="black" stroke-width="1" fill="none" stroke-linecap="round" />
        >>> cl2 = Closed_path(
        ...     path=p, offset=10,
        ...     stroke_width=1, color="red", ref_color="black"
        ... )
        >>> print(cl2.to_svg())
        <path d="M 10.000000 10.000000 L 190.000000 10.000000 L 190.000000 90.000000 L 110.000000 90.000000 A 20.000000 20.000000 0.000000 0 0 90.000000 110.000000 L 90.000000 190.000000 A 20.000000 20.000000 0.000000 0 0 110.000000 210.000000 L 190.000000 210.000000 L 190.000000 290.000000 L 10.000000 290.000000 L 10.000000 10.000000" stroke="red" stroke-width="1" fill="none"  stroke-linecap="round" />
        <path d="M 0.000000 0.000000 L 200.000000 0.000000 L 200.000000 100.000000 L 100.000000 100.000000 L 100.000000 200.000000 L 200.000000 200.000000 L 200.000000 300.000000 L 0.000000 300.000000 L 0.000000 0.000000" stroke="black" stroke-width="1" fill="none" stroke-linecap="round" />
        """
        if stroke_width is None:
            stroke_width = self._stroke_width
        if color is None:
            color = self._color
        if ref_color is None:
            ref_color = self._ref_color
        path = ""
        n = len( self._path )
        for i in range(n+1):
            seg_prev = Segment( self._path[(i-1)%n], self._path[(i)%n] )
            seg = Segment( self._path[(i)%n], self._path[(i+1)%n] )
            seg_off_prev = seg_prev.shifted(self._offset)
            seg_off = seg.shifted(self._offset)
            i1 = seg_off_prev.intersection( seg_off )
            if len(i1) == 1:
                if i==0:
                    prefix = "M"
                else:
                    prefix = " L"
                path += (prefix + " %f %f"%(i1[0].x, i1[0].y))
            else:
                seg_off_prev_2 = seg_prev.shifted(-self._offset)
                seg_off_2 = seg.shifted(-self._offset)
                i2 = seg_off_prev_2.intersection( seg_off_2 )
                if len(i2) == 0:
                    raise ValueError("offset is too important !")
                i1_1 = i2[0] + seg_off_prev.normal()*(2*self._offset)
                i1_2 = i2[0] + seg_off.normal()*(2*self._offset)
                if i==0:
                    path += "M %f %f"%(i1_2.x, i1_2.y)
                else:
                    path += " L %f %f"%(i1_1.x, i1_1.y)
                    large_arc_flag = 0
                    if self._offset>0:
                        sweep_flag = 0
                    else:
                        sweep_flag = 1
                    angle = 0
                    radius = abs(2*self._offset)
                    path += " A %f %f %f %d %d %f %f"%(
                        radius, radius, angle, large_arc_flag, sweep_flag,
                        i1_2.x, i1_2.y
                    )
        result = "<path d=\"%s\" stroke=\"%s\" stroke-width=\"%d\" fill=\"none\"  stroke-linecap=\"round\" />"%(
            path, color, stroke_width
        )
        if not ref_color is None:
            original_path = ""
            for i in range(n+1):
                if i == 0:
                    prefix = "M"
                else:
                    prefix = " L"
                p = self._path[i%n]
                original_path += (prefix + " %f %f"%(p.x, p.y))
            result += "\n<path d=\"%s\" stroke=\"%s\" stroke-width=\"%d\" fill=\"none\" stroke-linecap=\"round\" />"%(
                original_path, ref_color, stroke_width
            )
        return result

if __name__ == "__main__": 
    import doctest
    doctest.testmod()
