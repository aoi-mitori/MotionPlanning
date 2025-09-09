import math
import pprint
import copy
import tkinter as tk
import numpy as np
import time
from anytree import Node, RenderTree
from tkinter import font as tkFont


def rgb_hack(rgb):
    return "#%02x%02x%02x" % rgb 
    
####### canvas variable #######
CWIDTH = 500
CHEIGHT = 500
WWIDTH = 128
WHEIGHT = 128
Black = rgb_hack((0,0,0))
Pink = '#FEDFE1'
Gray = '#BDC0BA'
MIZU = '#58B2DC'
SORA = '#006284'
DarkBlue = '#0B346E'
max_pf_value = -1

####### window #######
window = tk.Tk()
window.title('Motion Planning')
window.geometry('700x540')
canvas = tk.Canvas(window, width=CWIDTH, height=CHEIGHT)
canvas.pack(padx=20, side="left")
canvas.configure(bg=Gray)

####### pf button #######
def DrawPf():
    pf_window = tk.Tk()
    pf_window.title('PF')
    pf_window.geometry('560x560')
    pf_canvas = tk.Canvas(pf_window, width=512, height=512)
    pf_canvas.pack(padx=20, pady=20)
    # for robot in my_init_robotArr:
    #     robot.transform()
    
    
    for robot in my_goal_robotArr:
        #robot.transform()
        for cp in robot.cpArr:
            cp.pf.fill(254)
            DrawObOnPf(cp)
        CreatePf( robot )
        # for cp in robot.cpArr:
        #     cp.pf.fill(254)
        #     draw_ob_on_pf( cp )
        #     create_pf( cp )
    #print(max_pf_value)
    for i in range(128):
        for j in range(128):
            if( my_goal_robotArr[0].cpArr[0].pf[i,j] == -1): color = 0
            else: color = 255- int( my_goal_robotArr[0].cpArr[0].pf[i,j] * 255/max_pf_value )
            if( color < 0): color = 0
            pf_canvas.create_rectangle( i*4, 128-(j+1)*4, (i+1)*4, (128-j)*4, fill=rgb_hack( (color, color, color) ) )
helv16 = tkFont.Font(family='Helvetica', size=16)
pf_button = tk.Button(window, text='Potential Field', command = DrawPf)
pf_button.pack(pady=20, padx=20)
pf_button['font'] = helv16
pf_button.config(width='20', height='2')

####### find path button #######
def FindPath():
    for robot in my_goal_robotArr:
    #robot.transform()
        for cp in robot.cpArr:
            cp.pf.fill(254)
            DrawObOnPf(cp)
        CreatePf( robot )
    for i in range( len( my_goal_robotArr) ):
        grobot = my_goal_robotArr[i]
        inirobot = my_init_robotArr[i]
        for j in range(len(grobot.cpArr)):
            inirobot.cpArr[j].pf = grobot.cpArr[j].pf
    for robot in my_init_robotArr:
        FindRobotPath( robot )
fp_button = tk.Button(window, text='Find Path', command = FindPath)
fp_button.pack(pady=20, padx=20)
fp_button['font'] = helv16
fp_button.config(width='20', height='2')


####### prototype #######
class State:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

class ControlPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.pf = np.zeros((128, 128))
        self.pf.fill(254)

class Obstacle:
    def __init__(self, convexArr, state, color):
        self.convexArr = convexArr
        self.state = state
        self.color = color

class Convex:
    def __init__(self, verticeArr):
        self.verticeArr = verticeArr
        self.obstacle_no = -1
        self.robot_no = -1
        self.id = 0
        self.color = Pink

class Vertex:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Robot:
    def __init__(self, convexArr, state, color, cpArr):
        self.convexArr = convexArr
        self.state = state
        self.color = color
        self.cpArr = cpArr
        self.tree = None

class MouseMover():
    def __init__(self):
        self.ids = []
        self.nos = []
        self.flag = -1
        self.previous = (0, 0)
        self.start = 0
        self.centers = []
        self.center = (0, 0)

    def select(self, event):
        widget = event.widget
        xc = widget.canvasx(event.x)
        yc = widget.canvasx(event.y)
        self.ids = []
        self.nos = []
        self.centers = []
        closest_id = widget.find_closest(xc, yc)[0]
        self.ids.append( closest_id )
        self.flag = 0
        no = -1
        flag = 0
        for i in range( len(my_obstacleArr) ):
            for j in range( len(my_obstacleArr[i].convexArr) ):
                if my_obstacleArr[i].convexArr[j].id == closest_id:
                    no = my_obstacleArr[i].convexArr[j].obstacle_no
                    flag = 1 # obstacle
                    self.nos.append( (i,j) )
                    self.flag = flag
                    break
        if flag == 0:
            for i in range( len(my_init_robotArr) ):
                for j in range( len(my_init_robotArr[i].convexArr) ):
                    if my_init_robotArr[i].convexArr[j].id == closest_id:
                        no = my_init_robotArr[i].convexArr[j].robot_no
                        flag = 2 # init_robot
                        self.nos.append( (i,j) )
                        self.flag = flag
                        break
        if flag == 0:
            for i in range( len(my_goal_robotArr) ):
                for j in range( len(my_goal_robotArr[i].convexArr) ):
                    if my_goal_robotArr[i].convexArr[j].id == closest_id:
                        no = my_goal_robotArr[i].convexArr[j].robot_no
                        flag = 3 # goal_robot
                        self.nos.append( (i,j) )
                        self.flag = flag
                        break

        if flag == 1: # obstacle
            for i in range( len(my_obstacleArr) ):
                for j in range( len(my_obstacleArr[i].convexArr) ):
                    if my_obstacleArr[i].convexArr[j].obstacle_no == no and my_obstacleArr[i].convexArr[j].id != self.ids[0]:
                        self.ids.append( my_obstacleArr[i].convexArr[j].id )
                        self.nos.append( (i,j) )
        elif flag == 2: # init_robot
            for i in range( len(my_init_robotArr) ):
                for j in range( len(my_init_robotArr[i].convexArr) ):
                    if my_init_robotArr[i].convexArr[j].robot_no == no and my_init_robotArr[i].convexArr[j].id != self.ids[0]:
                        self.ids.append( my_init_robotArr[i].convexArr[j].id )
                        self.nos.append( (i,j) )
        elif flag == 3: # goal_robot
            for i in range( len(my_goal_robotArr) ):
                for j in range( len(my_goal_robotArr[i].convexArr) ):
                    if my_goal_robotArr[i].convexArr[j].robot_no == no and my_goal_robotArr[i].convexArr[j].id != self.ids[0]:
                        self.ids.append( my_goal_robotArr[i].convexArr[j].id )
                        self.nos.append( (i,j) )
        self.previous = (xc, yc)

    def drag(self, event):
        widget = event.widget
        xc = widget.canvasx(event.x)
        yc = widget.canvasx(event.y)
        xx = xc-self.previous[0]
        yy = yc-self.previous[1]
        #print(xx, yy)
        window_xx = xx * WWIDTH / CWIDTH
        window_yy = yy * WHEIGHT / CHEIGHT
        ids = self.ids
        polygon = None
        convex = None
        if self.flag == 1: #obstacle
            my_obstacleArr[ self.nos[0][0] ].state.x += window_xx
            my_obstacleArr[ self.nos[0][0] ].state.y -= window_yy
            polygon = my_obstacleArr[ self.nos[0][0] ]
        elif self.flag == 2: #init_robot
            my_init_robotArr[ self.nos[0][0] ].state.x += window_xx
            my_init_robotArr[ self.nos[0][0] ].state.y -= window_yy
            polygon = my_init_robotArr[ self.nos[0][0] ]
        elif self.flag == 3: #goal_robot
            my_goal_robotArr[ self.nos[0][0] ].state.x += window_xx
            my_goal_robotArr[ self.nos[0][0] ].state.y -= window_yy
            polygon = my_goal_robotArr[ self.nos[0][0] ]

        for i in range( len(ids) ):
            convex = polygon.convexArr[ self.nos[i][1] ]
            points = []
            for vertex in convex.verticeArr:
                vertex_transformed = Transform( vertex, polygon.state )
                vertex_canvas = ChangeToCanvas( vertex_transformed )
                points.append( vertex_canvas.x )
                points.append( vertex_canvas.y )
            canvas.coords( ids[i], points) 
        self.previous = (xc, yc)
    
    def rotate(self, event):
        widget = event.widget
        xc = widget.canvasx(event.x)
        yc = widget.canvasx(event.y)
        my_angle = ( ( math.atan2(yc, xc) - math.atan2(self.previous[1], self.previous[0]) ) * 180 / math.pi )
        if( my_angle < 0): my_angle = 360 - abs( my_angle )
        #print(my_angle)

        polygon = None
        convex = None
        if self.flag == 1: #obstacle
            my_obstacleArr[ self.nos[0][0] ].state.r += my_angle
            while( my_obstacleArr[ self.nos[0][0] ].state.r >= 360):
                my_obstacleArr[ self.nos[0][0] ].state.r -= 360
            polygon = my_obstacleArr[ self.nos[0][0] ]
        elif self.flag == 2: #init_robot
            my_init_robotArr[ self.nos[0][0] ].state.r += my_angle
            while( my_init_robotArr[ self.nos[0][0] ].state.r >= 360):
                my_init_robotArr[ self.nos[0][0] ].state.r -= 360
            polygon = my_init_robotArr[ self.nos[0][0] ]
        elif self.flag == 3: #goal_robot
            my_goal_robotArr[ self.nos[0][0] ].state.r += my_angle
            while( my_goal_robotArr[ self.nos[0][0] ].state.r >= 360):
                my_goal_robotArr[ self.nos[0][0] ].state.r -= 360
            polygon = my_goal_robotArr[ self.nos[0][0] ]

        ids = self.ids
        for i in range( len(ids) ):
            convex = polygon.convexArr[ self.nos[i][1] ]
            points = []
            for vertex in convex.verticeArr:
                vertex_transformed = Transform( vertex, polygon.state )
                vertex_canvas = ChangeToCanvas( vertex_transformed )
                points.append( vertex_canvas.x )
                points.append( vertex_canvas.y )
            canvas.coords( ids[i], points) 


####### functions #######
def Transform(vertex, state):
    theta = state.r * math.pi / 180
    x = vertex.x
    y = vertex.y
    new_x = math.cos(theta) * x - math.sin(theta) * y + state.x
    new_y = math.sin(theta) * x + math.cos(theta) * y + state.y
    new_vertex = Vertex( new_x, new_y )
    return new_vertex

def ChangeToCanvas(vertex):
    new_x = vertex.x * CWIDTH / WWIDTH
    new_y = CHEIGHT - vertex.y * CHEIGHT / WHEIGHT
    new_vertex = Vertex( new_x, new_y )
    return new_vertex

def DrawPolygon( polygon, state, color ):
    for convex in polygon.convexArr:
        points = []
        for vertex in convex.verticeArr:
            vertex_transformed = Transform( vertex, state)
            vertex_canvas = ChangeToCanvas( vertex_transformed )
            points.append( vertex_canvas.x )
            points.append( vertex_canvas.y )
        convex.id = canvas.create_polygon( points, fill=color )

def DrawObOnPf( cp ):
    points = []
    for obstacle in my_obstacleArr:
        points = []
        for convex in obstacle.convexArr:
            for vertex in convex.verticeArr:
                vertex_transformed = Transform( vertex, obstacle.state )
                points.append( vertex_transformed.x )
                points.append( vertex_transformed.y )
            points.append( points[0] )
            points.append( points[1] )
            for i in range( 0, len(points)-2, 2):
                x1 = points[i]
                y1 = points[i+1]
                x2 = points[i+2]
                y2 = points[i+3]
                d = int( max( abs(x2-x1), abs(y2-y1) ) )+1
                dx =  (x2-x1) / d 
                dy =  (y2-y1) / d  
                for j in range(d+1):
                    xi = int( x1+j*dx )
                    yi = int( y1+j*dy )
                    if xi<128 and xi>=0 and yi<128 and yi>=0 :
                        cp.pf[xi, yi] = -1            

def CreatePf( robot ):
    for cp in robot.cpArr:
        record = np.zeros((128, 128))
        queue = []
        cp_transformed = Transform( cp, robot.state )
        queue.append( ( int(cp_transformed.x), int(cp_transformed.y), 0) )
        while( len(queue) > 0 ):
            first, queue = queue[0], queue[1:]
            x, y, n = first[0], first[1], first[2]
            cp.pf[x,y] =  n
            global max_pf_value 
            max_pf_value = n+1
            if x+1<128 and cp.pf[x+1,y] == 254 and record[x+1,y] == 0:
                queue.append( (x+1, y, n+1) )
                record[x+1,y] = 1
            if x-1>=0 and cp.pf[x-1,y] == 254 and record[x-1,y] == 0:
                queue.append( (x-1, y, n+1) )
                record[x-1, y] = 1
            if y+1<128 and cp.pf[x,y +1] == 254 and record[x,y+1] == 0:
                queue.append( (x, y+1, n+1) )
                record[x, y+1] = 1
            if y-1>=0 and cp.pf[x, y-1] == 254 and record[x,y-1] == 0:
                queue.append( (x, y-1, n+1) )
                record[x, y-1] = 1

def GetPfValue( state, cpArr ):
    pfv = 0
    for cp in cpArr:
        cp_transformed = Transform( cp, state )
        x = int(cp_transformed.x)
        y = int(cp_transformed.y)
        pfv += cp.pf[x][y]
    return pfv

def FindMaxMinVertex( convex, state ):
    max_vertex = None
    min_vertex = None
    for i in range( len(convex.verticeArr) ):
        vertex_transformed = Transform( convex.verticeArr[i], state)
        if( i==0 ):
            max_vertex = copy.deepcopy( vertex_transformed )
            min_vertex = copy.deepcopy( vertex_transformed )
        else:
            if( vertex_transformed.x > max_vertex.x ): max_vertex.x = vertex_transformed.x
            if( vertex_transformed.y > max_vertex.y ): max_vertex.y = vertex_transformed.y
            if( vertex_transformed.x < min_vertex.x ): min_vertex.x = vertex_transformed.x
            if( vertex_transformed.y < min_vertex.y ): min_vertex.y = vertex_transformed.y
    return (max_vertex.x, max_vertex.y, min_vertex.x, min_vertex.y)

def Colli( convex1, convex2, state1, state2 ):
    max_x_1, max_y_1, min_x_1, min_y_1 = FindMaxMinVertex( convex1, state1 )
    max_x_2, max_y_2, min_x_2, min_y_2 = FindMaxMinVertex( convex2, state2 )
    if( max_x_1 >=128 or min_x_1 <0 or max_y_1>=128 or min_y_1<0 ): return 0 #out of bound
    elif( max_x_1 < min_x_2 or max_y_1 < min_y_2 or min_x_1 > max_x_2 or min_y_1 > max_y_2 ): 
        return 1 # no collision
    else: return 2 # maybe collision

def Dot( v1, v2 ):
    return ( v1[0]*v2[0] + v1[1]*v2[1] )

def CrossDetection(p1, p2, p3, p4, state1, state2):
    p1_transformed = Transform( p1, state1)
    p2_transformed = Transform( p2, state1)
    p3_transformed = Transform( p3, state2)
    p4_transformed = Transform( p4, state2)

    v12 = ( p2_transformed.x-p1_transformed.x, p2_transformed.y-p1_transformed.y)
    v12_ = ( -1*v12[1], v12[0] )
    v13 = ( p3_transformed.x-p1_transformed.x, p3_transformed.y-p1_transformed.y)
    v14 = ( p4_transformed.x-p1_transformed.x, p4_transformed.y-p1_transformed.y)
    v34 = ( p3_transformed.x-p4_transformed.x, p3_transformed.y-p4_transformed.y)
    v34_ = ( -1*v34[1], v34[0] )
    v31 = ( p3_transformed.x-p1_transformed.x, p3_transformed.y-p1_transformed.y)
    v32 = ( p3_transformed.x-p2_transformed.x, p3_transformed.y-p2_transformed.y)

    if( ( Dot(v12_, v13) * Dot(v12_, v14) <0 ) and ( Dot(v34_, v31) * Dot(v34_, v32) <0 ) ):
        return True # collision
    else: return False

def Collision( robot, state, obstacle):
    for r_convex in robot.convexArr:
        for o_convex in obstacle.convexArr:
            collii = Colli( r_convex, o_convex, state, obstacle.state )
            if( collii == 0 ): 
                #print("1!!")
                return True # out of bound -> collision
            elif( collii == 1 ): continue # no collision -> check next one
            else: # maybe collision -> check cross
                r_vertexs = copy.deepcopy(r_convex.verticeArr)
                r_vertexs.append( r_convex.verticeArr[0] )
                o_vertexs = copy.deepcopy(o_convex.verticeArr)
                o_vertexs.append( o_convex.verticeArr[0] )
                for i in range( len(r_vertexs)-1 ):
                    r_vertex_1 = r_vertexs[i]
                    r_vertex_2 = r_vertexs[i+1]
                    for j in range( len(o_vertexs)-1 ):    
                        o_vertex_1 = o_vertexs[j]
                        o_vertex_2 = o_vertexs[j+1]
                        if( CrossDetection( r_vertex_1, r_vertex_2, o_vertex_1, o_vertex_2, state, obstacle.state )== True):
                            return True
                        
    return False                   

def DrawCovex( convex, state, color):
    points = []
    for vertex in convex.verticeArr:
        vertex_transformed = Transform( vertex, state)
        vertex_canvas = ChangeToCanvas( vertex_transformed )
        points.append( vertex_canvas.x )
        points.append( vertex_canvas.y )
    idd = canvas.create_polygon( points, fill=color )
    return idd

def MoveConvex( id, convex, state):
    points = []
    for vertex in convex.verticeArr:
        vertex_transformed = Transform( vertex, state)
        vertex_canvas = ChangeToCanvas( vertex_transformed )
        points.append( vertex_canvas.x )
        points.append( vertex_canvas.y )
    canvas.coords(id, points) 

def MovePoly(robot, state, ids):
    for i in range( len(robot.convexArr) ):
        MoveConvex( ids[i], robot.convexArr[i], state)

def FindRobotPath( robot ):
    visited_arr = [[[False for k in range(36)] for j in range(128)] for i in range(128)]
    opened_arrlist = []
    node_arrlist = []
    for i in range(600):
        opened_arrlist.append([])
        node_arrlist.append([])
    pfv = int(GetPfValue( robot.state, robot.cpArr ))
    opened_arrlist[ pfv ].append( robot.state)
    node_arrlist[ pfv ].append( Node( robot.state) )

    visited_arr[ int(robot.state.x) ][ int(robot.state.y) ][ int(robot.state.r//10) ] = True

    global lastNode
    found = False
    while( found == False ):
        state_now = None
        for i in range(600):
            if( len(opened_arrlist[i])!=0 ):
                print(i)
                state_now, opened_arrlist[i] = opened_arrlist[i][0], opened_arrlist[i][1:] #pop
                node_now, node_arrlist[i] = node_arrlist[i][0], node_arrlist[i][1:]
                #node_now = Node( state_now )
                if( robot.tree is None ): 
                    robot.tree =  node_now 
                    # print("======")
                    # for pre, fill, node in RenderTree(robot.tree):
                    #     print("%s%s" % (pre, node.name))
                break
        if( state_now is not None ):
            neighbors = []
            neighbors.append( State(state_now.x+1, state_now.y, state_now.r) )
            neighbors.append( State(state_now.x-1, state_now.y, state_now.r) )
            neighbors.append( State(state_now.x, state_now.y+1, state_now.r) )
            neighbors.append( State(state_now.x, state_now.y-1, state_now.r) )
            neighbors.append( State(state_now.x, state_now.y, state_now.r+10) )
            neighbors.append( State(state_now.x, state_now.y, state_now.r-10) )
            
            for neighbor in neighbors:
                if( neighbor.r >= 360): neighbor.r -= 360
                elif ( neighbor.r <0 ): neighbor.r += 360
                
                add_flag = True
                if( int(neighbor.x) < 0 or int(neighbor.x) >=128 or int(neighbor.y) < 0 or int(neighbor.y) >= 128): 
                    add_flag = False #over table
                else: 
                    if( visited_arr[ int(neighbor.x) ][ int(neighbor.y) ][ int(neighbor.r//10)] == False ):
                        visited_arr[ int(neighbor.x) ][ int(neighbor.y) ][ int(neighbor.r//10)] = True
                        for obstacle in my_obstacleArr :
                            if( Collision( robot, neighbor, obstacle ) == True ):
                                add_flag = False
                                break
                    else: add_flag = False  
                if( add_flag == True ):
                    index = int(GetPfValue( neighbor, robot.cpArr ))
                    #DrawPolygon( robot, neighbor, "#D7C4BB")
                    opened_arrlist[ index ].append( neighbor )

                    lastNode = Node( neighbor, parent = node_now )
                    node_arrlist[ index ].append( lastNode )
                    #print(node_now)
                    #print("----")
                    
                    # for pre, fill, node in RenderTree(robot.tree):
                    #     print("%s%s" % (pre, node.name))
                    if( index == 0 ): 
                        found = True
                        break
        else: break
    if( found == True ): 
        # for pre, fill, node in RenderTree(robot.tree):
        #     print("%s%s" % (pre, node.name.x))
        print("found")
        my_path = []
        #print(lastNode)
        while( lastNode != robot.tree ):
            my_path.append( lastNode.name )
            lastNode = lastNode.parent
        
        my_ids = []
        for convex in robot.convexArr:
            my_ids.append( DrawCovex( convex, my_path[0], "#FFFFFB") ) #draw last one

        for state in my_path:
            MovePoly(robot, state, my_ids)
            window.update()
            time.sleep(0.005)
 
        #print(my_ids)

        # my_poly = DrawPolygon( robot, my_path[0], "#FFFFFB")
        for i in range(len(my_path)):
            if( i%8 == 0):
                DrawPolygon( robot, my_path[i], "#FFFFFB")
            
        #print( my_path)

    else : print( "not found")



####### read obstacle file #######
f = open("bfp-data/obstacle5.dat", "r")
my_obstacleArr = []
lines = f.readlines()
l = 0
while l<len(lines):
    line = lines[l]
    while line[0] == '#' or line[0] == ' ':
        l += 1
        line = lines[l]
    obstacle_num = int(line.split()[0])
    l += 1
    line = lines[l]
    for ob in range(obstacle_num):
        line = lines[l]
        while line[0] == '#' or line[0] == ' ':
            l += 1
            line = lines[l]
        convex_num = int(line.split()[0])
        my_convexArr = []
        l += 1
        line = lines[l]
        for cov in range(convex_num):
            while line[0] == '#' or line[0] == ' ':
                l += 1
                line = lines[l]
            vertice_num = int(line.split()[0])
            my_vertexArr = []
            l += 1
            line = lines[l]
            for vertice in range(vertice_num):
                while line[0] == '#' or line[0] == ' ':
                    l += 1
                    line = lines[l]
                x = float(line.split()[0])
                y = float(line.split()[1])
                l += 1
                line = lines[l]
                my_vertex = Vertex(x,y)
                my_vertexArr.append(my_vertex)
            my_convex = Convex(my_vertexArr)
            my_convex.obstacle_no = ob
            my_convexArr.append(my_convex)
        while line[0] == '#' or line[0] == ' ':
            l += 1
            line = lines[l]
        conf_x = float(line.split()[0])
        conf_y = float(line.split()[1])
        conf_r = float(line.split()[2])
        conf = State(conf_x, conf_y, conf_r)
        my_obstacle = Obstacle(my_convexArr, conf, Black)
        my_obstacleArr.append(my_obstacle)
        l += 1
        
####### read robot file #######
f = open("bfp-data/robot5.dat", "r")
my_init_robotArr = []
my_goal_robotArr = []
lines = f.readlines()
l = 0
while l<len(lines):
    line = lines[l]
    while line[0] == '#' or line[0] == ' ':
        l += 1
        line = lines[l]
    robot_num = int(line.split()[0])
    l += 1
    line = lines[l]
    for robot in range(robot_num):
        line = lines[l]
        while line[0] == '#' or line[0] == ' ':
            l += 1
            line = lines[l]
        convex_num = int(line.split()[0])
        my_convexArr = []
        l += 1
        line = lines[l]
        for cov in range(convex_num):
            while line[0] == '#' or line[0] == ' ':
                l += 1
                line = lines[l]
            vertice_num = int(line.split()[0])
            my_vertexArr = []
            l += 1
            line = lines[l]
            for vertice in range(vertice_num):
                while line[0] == '#' or line[0] == ' ':
                    l += 1
                    line = lines[l]
                x = float(line.split()[0])
                y = float(line.split()[1])
                l += 1
                line = lines[l]
                my_vertex = Vertex(x,y)
                my_vertexArr.append(my_vertex)
            my_convex = Convex(my_vertexArr)
            my_convex.robot_no = robot
            my_convexArr.append(my_convex)
        while line[0] == '#' or line[0] == ' ':
            l += 1
            line = lines[l]
        conf_x = float(line.split()[0])
        conf_y = float(line.split()[1])
        conf_r = float(line.split()[2])
        l += 1
        line = lines[l]
        init_conf = State(conf_x, conf_y, conf_r)
        while line[0] == '#' or line[0] == ' ':
            l += 1
            line = lines[l]
        conf_x = float(line.split()[0])
        conf_y = float(line.split()[1])
        conf_r = float(line.split()[2])
        l += 1
        line = lines[l]
        goal_conf = State(conf_x, conf_y, conf_r)
        while line[0] == '#' or line[0] == ' ': 
            l += 1
            line = lines[l]
        my_cpArr = []
        cp_num = int(line.split()[0])
        l += 1
        line = lines[l]
        for cp in range(cp_num):
            while line[0] == '#' or line[0] == ' ':
                l += 1
                line = lines[l]
            cp_x = int(float(line.split()[0]))
            cp_y = int(float(line.split()[1]))
            cp = ControlPoint(cp_x, cp_y)
            l += 1
            if(l < len(lines)):
                line = lines[l]
            my_cpArr.append(cp)
        my_convexArr1 = copy.deepcopy( my_convexArr )
        my_init_robot = Robot(my_convexArr, init_conf, MIZU, my_cpArr)
        my_init_robotArr.append(my_init_robot)
        my_cpArr1 = copy.deepcopy(my_cpArr)
        my_goal_robot = Robot(my_convexArr1, goal_conf, SORA, my_cpArr1)
        my_goal_robotArr.append(my_goal_robot)

####### draw polygons #######
for obstacle in my_obstacleArr:
    DrawPolygon( obstacle, obstacle.state, obstacle.color )
for robot in my_init_robotArr:
    DrawPolygon( robot, robot.state, robot.color )
for robot in my_goal_robotArr:
    DrawPolygon( robot, robot.state, robot.color )

mm = MouseMover()
canvas.bind("<Button-1>", mm.select)
canvas.bind("<B1-Motion>", mm.drag)
canvas.bind("<Button-3>", mm.select)
canvas.bind("<B3-Motion>", mm.rotate)

window.mainloop()        