import geometry
import math
import doctest
import turtle

def left_foot( origin, center_x, center_y, width, height, radius, offset, ref_color=None ):
    res = [] 
    res.append(
        geometry.Circle( origin, radius, offset=offset, ref_color=ref_color )
    )
    res.append(
        geometry.Circle( origin + geometry.Vec2d(width, 0), radius, offset=offset, ref_color=ref_color )
    )
    res.append(
        geometry.Circle( origin + geometry.Vec2d(width, height), radius, offset=offset, ref_color=ref_color )
    )
    res.append(
        geometry.Circle( origin + geometry.Vec2d(0, height), radius, offset=offset, ref_color=ref_color )
    )

    # Print the center
    center_size = 0.5
    res.append(
        geometry.Circle(
            origin, center_size, offset=offset, color="black", 
            ref_color=ref_color, fill_color="black"
        )
    )

    return res

class Piece:
    def __init__(self, size_aruco, a, b, c, d, e, f, g, h, i, j, k, l):
        self.s =size_aruco
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.e = e
        self.f = f
        self.g = g
        self.h = h
        self.i = i
        self.j = j
        self.k = k
        self.l = l

    def prise_male_femelle(self, t, path, male=1):
        def turn(t, val):
            if( val==1 ):
                t.left()
            else:
                t.right()
        turn(t, male)
        path.append(t.forward(self.a/2))
        turn(t, male)
        path.append(t.forward((self.b-self.c)/2))
        turn(t, -male)
        path.append(t.forward(self.a/2))
        turn(t, -male)
        path.append(t.forward(self.b))
        turn(t, -male)
        path.append(t.forward(self.a/2))
        turn(t, -male)
        path.append(t.forward((self.b-self.c)/2))
        turn(t, male)
        path.append(t.forward(self.a/2))
        turn(t, male)

    def les_prises(self, t, path):
        path.append(t.forward(self.d))
        self.prise_male_femelle(t, path, male=1)
        path.append(t.forward(self.e))
        self.prise_male_femelle(t, path, male=-1)
        path.append(t.forward(self.d))

    def les_prises_encastre(self, t, path):
        t.left()
        path.append(t.forward(self.a))
        t.right()
        self.les_prises(t, path)
        t.right()
        path.append(t.forward(self.a))
        t.left()

    def piece_height(self, t, path, accroche=False):
        if not accroche:
            path.append( t.forward(self.s+self.h) )
        else:
            path.append( t.forward(self.s) )
            t.left()
            path.append( t.forward(self.k) )
            t.right()
            path.append( t.forward(self.l) )
            t.right()
            path.append( t.forward(self.k) )
            t.left()
            path.append( t.forward(self.h-self.l) )
        self.les_prises_encastre(t, path)
        if not accroche:
            path.append( t.forward(self.s+self.h) )
        else:
            path.append( t.forward(self.h-self.l) )
            t.left()
            path.append( t.forward(self.k) )
            t.right()
            path.append( t.forward(self.l) )
            t.right()
            path.append( t.forward(self.k) )
            t.left()
            path.append( t.forward(self.s) )

    def piece_width(self, t, path, bottom_to_top=False, accroche=False):
        if bottom_to_top :
            m = (self.g-self.s)/2
            if not accroche:
                path.append( t.forward(self.g) )
            else:
                path.append( t.forward(self.g-self.h-m) )
                t.left()
                path.append( t.forward(self.k) )
                t.right()
                path.append( t.forward(self.l) )
                t.right()
                path.append( t.forward(self.k) )
                t.left()
                path.append( t.forward(self.h-self.l+m) )
            self.les_prises_encastre(t, path)
            if not accroche:
                path.append( t.forward(self.s) )
            else:
                path.append( t.forward(self.h-self.l-m) )
                t.left()
                path.append( t.forward(self.k) )
                t.right()
                path.append( t.forward(self.l) )
                t.right()
                path.append( t.forward(self.k) )
                t.left()
                path.append( t.forward(self.s-self.h+m) )
        else:
            m = (self.s-self.g)/2
            if not accroche:
                path.append( t.forward(self.s) )
            else:
                path.append( t.forward(self.s-self.h-m) )
                t.left()
                path.append( t.forward(self.k) )
                t.right()
                path.append( t.forward(self.l) )
                t.right()
                path.append( t.forward(self.k) )
                t.left()
                path.append( t.forward(self.h-self.l+m) )
            self.les_prises_encastre(t, path)
            if not accroche:
                path.append( t.forward(self.g) )
            else:
                path.append( t.forward(self.h-self.l-m) )
                t.left()
                path.append( t.forward(self.k) )
                t.right()
                path.append( t.forward(self.l) )
                t.right()
                path.append( t.forward(self.k) )
                t.left()
                path.append( t.forward(self.g-self.h+m) )

    def path_of_piece(self, origin, accroche=False):
        t = turtle.Turtle( origin, geometry.Vec2d(1,0) )
        path = [origin]

        self.piece_height(t, path, accroche=accroche)
        t.left()
        self.piece_width(t, path, bottom_to_top=True, accroche=accroche)
        t.left()
        self.piece_height(t, path, accroche=accroche)
        t.left()
        self.piece_width(t, path, bottom_to_top=False, accroche=accroche)
        t.left()

        return path[:-1]

    def path_of_sub_piece(self, origin):
        t = turtle.Turtle( origin, geometry.Vec2d(1,0) )
        path = [origin]

        for i in range(2):
            path.append( t.forward(self.f) )
            t.left()
            self.les_prises(t, path)
            t.left()

        return path[:-1]

    def accroche(self, origin):
        t = turtle.Turtle( origin, geometry.Vec2d(1,0) )
        path = [origin]

        t.left()
        self.piece_width(t, path, bottom_to_top=True, accroche=True)
        t.left()
        path.append( t.forward(3) )
        t.left()
        path.append( t.forward(width_piece) )
        t.left()
        path.append( t.forward(3) )

        return path[:-1]
    def base(self, origin):
        t = turtle.Turtle( origin, geometry.Vec2d(1,0) )
        path = [origin]

        path.append( t.forward(height_piece) )
        t.left()
        self.piece_width(t, path, bottom_to_top=True)
        t.left()
        self.piece_height(t, path)
        t.left()
        self.piece_width(t, path, bottom_to_top=False)
        t.left()

        return path[:-1]


#################################### Constants ################################

foot_radius = 12.3/2
offset = 0.12


small_foot_center_x = 31.5
small_foot_center_y = 50.5
small_foot_width = 71
small_foot_height = 111

big_foot_center_x = 32.5
big_foot_center_y = 55.304
big_foot_width = 76
big_foot_height = 141.609
big_foots_separation = 74

origin = geometry.Vec2d(0,0)

debug = None # "black" # Set to None to remove debuging mode and 'black' to activate the debuging mode.

size_aruco = 100
taille_encastrement = 140
cou_piece = 15
tete_piece = 35
profondeur_encastrement = 15
separation_prises = ( taille_encastrement - 2 * cou_piece )/3
taille_bas = 60
separation_aruco_encastrement = 50
taille_jonctions = 240

profondeur_accroche = 20
largeur_accroche = 5

taille_les_deux_prises = 2*cou_piece + 3*separation_prises
width_piece = size_aruco + taille_bas + taille_les_deux_prises
height_piece = 2*size_aruco + 2*separation_aruco_encastrement + taille_les_deux_prises


trou_x = 130 
trou_y = width_piece - size_aruco

montant_interieur = (height_piece - taille_jonctions)/2

origine_jonction = geometry.Vec2d(
    montant_interieur, (width_piece - taille_les_deux_prises)/2 - 25
)

############################### Plan ##########################################

d = geometry.Drawing("Robot", "Description")

p = Piece(
    size_aruco = size_aruco,
    a=profondeur_encastrement, b=tete_piece, c=cou_piece, 
    d=separation_prises, e=separation_prises, f=taille_jonctions, 
    g=taille_bas, h=separation_aruco_encastrement, i=trou_x, j=trou_y,
    k=profondeur_accroche, l=largeur_accroche
)

" Choose piece "
outputs=["four_base_pieces","base","accroche"]

output = outputs[0]
if output == "accroche":
    d.add(
        geometry.Closed_path(
            path=p.accroche(geometry.Vec2d(0,0)), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )

elif output == "base":
    d.add(
        geometry.Closed_path(
            path=p.base(geometry.Vec2d(0,0)), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )
    dist=(height_piece-2*big_foot_width-big_foots_separation)/2
    d.add(
        left_foot(
                origin = geometry.Vec2d(
                    height_piece-dist-big_foot_width,
                    width_piece/3-60
                ),
                center_x = big_foot_center_x, center_y = big_foot_center_y , 
                width = big_foot_width, height = big_foot_height, 
                radius = foot_radius, offset = offset, ref_color = debug
        )
    )
    d.add(
        left_foot(
                origin = geometry.Vec2d(
                    dist, width_piece/3 - 60
                ),
                center_x = big_foot_center_x, center_y = big_foot_center_y , 
                width = big_foot_width, height = big_foot_height, 
                radius = foot_radius, offset = offset, ref_color = debug
        )
    )

elif output == "four_base_pieces":
    space=4
    d.add(
        geometry.Closed_path(
            path=p.path_of_piece(geometry.Vec2d(0,0),accroche=True), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )

    d.add(
        geometry.Closed_path(
            path=p.path_of_sub_piece(origine_jonction), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )

    trans=geometry.Vec2d(space+height_piece,0)
    d.add(
        geometry.Closed_path(
            path=p.path_of_piece(geometry.Vec2d(0,0)+trans,accroche=True), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )

    d.add(
        geometry.Closed_path(
            path=p.path_of_sub_piece(origine_jonction+trans), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )
    trans=geometry.Vec2d(0,space+width_piece)
    d.add(
        geometry.Closed_path(
            path=p.path_of_piece(geometry.Vec2d(0,0)+trans,accroche=True), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )

    d.add(
        geometry.Closed_path(
            path=p.path_of_sub_piece(origine_jonction+trans), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )
    trans=geometry.Vec2d(space+height_piece,space+width_piece)
    d.add(
        geometry.Closed_path(
            path=p.path_of_piece(geometry.Vec2d(0,0)+trans,accroche=True), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )
    d.add(
        geometry.Closed_path(
            path=p.path_of_sub_piece(origine_jonction+trans), offset=-offset,
            stroke_width=1, color="red", ref_color=debug
        )
    )
#d.add(
#    geometry.Closed_path(
#        path=p.path_of_piece(geometry.Vec2d(height_piece + 10,0),accroche=False), offset=-offset,
#        stroke_width=1, color="red", ref_color=debug
#    )
#)


#origin_base_pour_grand_robot = geometry.Vec2d(0,width_piece + 10)
#d.add(
#    geometry.Closed_path(
#        path=p.base(origin_base_pour_grand_robot, accroche=True), offset=-offset,
#        stroke_width=1, color="red", ref_color=debug
#    )
#)

#d.add(
#    left_foot(
#            origin = origin_base_pour_petit_robot+geometry.Vec2d(
#                height_piece/2, width_piece/3 - 8
#            ), 
#            center_x = big_foot_center_x, center_y = big_foot_center_y , 
#            width = big_foot_width, height = big_foot_height, 
#            radius = foot_radius, offset = offset, ref_color = debug
#    )
#)

print( d.to_svg() )

