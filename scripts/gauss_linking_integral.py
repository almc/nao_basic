# import pdb; pdb.set_trace()
import sys, itertools, Image
import numpy as np
try:
    from OpenGL.GLUT import *
    from OpenGL.GL   import *
    from OpenGL.GLU  import *
except:
    print '''
ERROR: PyOpenGL not installed properly. sudo apt-get install python-opengl
        '''
    sys.exit()

################################################################################
print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
window_width  = 1200
window_height = 800
window_center = np.array([float(window_width)/2, float(window_height)/2])
# camera parameters (position, rotation)
xpos = 1.0; ypos = 2.0; zpos = 5.0; xrot = 0.0; yrot = 0.0; zrot = 0.0
texture  = 0; textureWall = 1

################################################################################
#                                 2D data                                      #
################################################################################
n_points      = [2, 4]                        # number of points for each strand
n_max_points  = np.max(n_points)               # maximum number of points fall s
n_strands     = np.size(n_points)                            # number of strands
strands       = range(n_strands)                        # enumeration of strands
points        = np.zeros([n_strands, n_max_points, 3])         # 3 coord (x,y,z)
connect       = np.zeros([n_strands, n_max_points, n_max_points])  # adjancecy m
################################################################################
## points
# s0
points[0,0,:] = [-2, 1, 0]
points[0,1,:] = [ 2, 1, 0]
# s1
points[1,0,:] = [ 0, 2, 2]
points[1,1,:] = [ 0, 0, 2]
points[1,2,:] = [ 0, 0,-2]
points[1,3,:] = [ 0, 2,-2]
## connectivity
# s0
connect[0,0,1] = connect[0,1,0] = 1
# s1
connect[1,0,1] = connect[1,1,0] = 1
connect[1,1,2] = connect[1,2,1] = 1
connect[1,2,3] = connect[1,3,2] = 1
print "--> points\n", points
print "--> connect\n", connect
################################################################################

# ################################################################################
# #                                 3D data                                      #
# ################################################################################
# n_points      = [2, 4, 3]                     # number of points for each strand
# n_max_points  = np.max(n_points)               # maximum number of points fall s
# n_strands     = np.size(n_points)                            # number of strands
# strands       = range(n_strands)                        # enumeration of strands
# points        = np.zeros([n_strands, n_max_points, 3])         # 3 coord (x,y,z)
# connect       = np.zeros([n_strands, n_max_points, n_max_points])  # adjancecy m
# ################################################################################
# ## points
# # s0
# points[0,0,:] = [-2, 1, 0]
# points[0,1,:] = [ 2, 1, 0]
# # s1
# points[1,0,:] = [ 0, 2, 2]
# points[1,1,:] = [ 0, 0, 2]
# points[1,2,:] = [ 0, 0,-2]
# points[1,3,:] = [ 0, 2,-2]
# # s3
# points[2,0,:] = [-2, 1,-1]
# points[2,1,:] = [ 2, 1,-1]
# points[2,2,:] = [ 2, 1, 1]
# ## connectivity
# # s0
# connect[0,0,1] = connect[0,1,0] = 1
# # s1
# connect[1,0,1] = connect[1,1,0] = 1
# connect[1,1,2] = connect[1,2,1] = 1
# connect[1,2,3] = connect[1,3,2] = 1
# # s3
# connect[2,0,1] = connect[2,1,0] = 1
# connect[2,1,2] = connect[2,2,1] = 1
# print "--> points\n", points
# print "--> connect\n", connect
# ################################################################################

################################################################################
# gauss linking integral
n_lines = np.zeros(n_strands)
for s in range(n_strands):
    n_lines[s] = np.sum(connect[s,:,:])/2         # summing all matrix yields 2x
n_lines = n_lines.astype(int)                            # convert double to int
n_max_lines = np.max(n_lines)                          # used to cover all cases
i_lines = np.zeros((n_strands, n_max_lines, 2))     # 2 initial/final coordinate
for s in range(n_strands):
    c = 0                                    # counter to represent current line
    for i in range(n_points[s]):
        for j in range(n_points[s]):
            if j >= i:           # loop only through the lower triangular strict
                break
            else:
                if connect[s,i,j] == 1:
                    i_lines[s,c,:] = [j,i]       # i_lines has initial/final j/i
                    c += 1                      # counter moves to the next line
print "--> n_lines\n", n_lines
print "--> i_lines\n", i_lines
################################################################################
# writhe matrix
# strand_combos_it = itertools.combinations(strands, 2) # combine strands 2 by 2
strand_combos_it = itertools.combinations_with_replacement(strands, 2)
strand_combos = []
for it in strand_combos_it:
    strand_combos.append(it)
strand_combos = np.asarray(strand_combos)
print "--> strand combinations\n", strand_combos

writhe_matrix = []
for sc in strand_combos:
    writhe_matrix.append(np.zeros( [n_lines[sc[0]], n_lines[sc[1]]] ))
    # print "--> scombo\n", sc, np.zeros( [n_lines[sc[0]], n_lines[sc[1]]] )
writhe_matrix = np.asarray(writhe_matrix)
print "--> init writhe_matrix\n", writhe_matrix
print "--> shape writhe_matrix\n", writhe_matrix.shape

def writhe_n():
    for s0 in range(n_strands):
        for s1 in range(n_strands):
            if s1 > s0:                       # avoiding duplicated calculations
                break
            else:
                print "calculating writhe between strands:", s1, s0
                print "lines per strand:", n_lines[s1], n_lines[s0]
                writhe_2(s1, s0)

def writhe_2(s0, s1):
    assert(s0 <= s1)
    # s0 = 1; s1 = 0
    # idx_sc = np.argwhere(np.all((strand_combos-np.array([s0,s1]))==0,axis=-1))
    idx_sc = np.argwhere((strand_combos == np.array([s0,s1])).all(-1))
    print "index_strand_combo", idx_sc
    for l0 in range(n_lines[s0]):
        for l1 in range(n_lines[s1]):
            idx_l0 = i_lines[s0,l0,:]; idx_l1 = i_lines[s1,l1,:]
            a = points[s0,idx_l0[0],:]; b = points[s0,idx_l0[1],:]
            c = points[s1,idx_l1[0],:]; d = points[s1,idx_l1[1],:]
            r_ab = b - a; r_ac = c - a; r_ad = d - a;
            r_bc = c - b; r_bd = d - b; r_cd = d - c
            n_a  = np.cross(r_ac, r_ad); n_an = np.linalg.norm(n_a)
            if n_an != 0: n_a = n_a / n_an
            n_b  = np.cross(r_ad, r_bd); n_bn = np.linalg.norm(n_b)
            if n_bn != 0: n_b = n_b / n_bn
            n_c  = np.cross(r_bd, r_bc); n_cn = np.linalg.norm(n_c)
            if n_cn != 0: n_c = n_c / n_cn
            n_d  = np.cross(r_bc, r_ac); n_dn = np.linalg.norm(n_d)
            if n_dn != 0: n_d = n_d / n_dn
            writhe_matrix.item(idx_sc)[l0,l1] = np.arcsin(np.dot(n_a,n_b)) + \
                                                np.arcsin(np.dot(n_b,n_c)) + \
                                                np.arcsin(np.dot(n_c,n_d)) + \
                                                np.arcsin(np.dot(n_d,n_a))



writhe_n()
print "--> writhe_matrix\n", '\n'.join(map(str, writhe_matrix))

def initGL(width, height):
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glDepthFunc(GL_LESS)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)

    glMatrixMode(GL_MODELVIEW)
    # glEnable(GL_TEXTURE_2D)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
    glutSetCursor(GLUT_CURSOR_NONE)

def draw_coordinate():
    glBegin(GL_LINES)
    glColor3f(0.0, 1.0, 0.0)    # Green for x axis
    glVertex3f(0,0,0)
    glVertex3f(10,0,0)
    glColor3f(1.0,0.0,0.0)      # Red for y axis
    glVertex3f(0,0,0)
    glVertex3f(0,10,0)
    glColor3f(0.0,0.0,1.0)      # Blue for z axis
    glVertex3f(0,0,0)
    glVertex3f(0,0,10)
    glEnd()

    glEnable(GL_LINE_STIPPLE)
    glLineStipple(1, 0x0101)
    glBegin(GL_LINES)
    glColor3f(0.0, 1.0, 0.0)    # Green for -x axis
    glVertex3f(-10,0,0)
    glVertex3f(0,0,0)
    glColor3f(1.0,0.0,0.0)      # Red for -y axis
    glVertex3f(0,0,0)
    glVertex3f(0,-10,0)
    glColor3f(0.0,0.0,1.0)      # Blue for -z axis
    glVertex3f(0,0,0)
    glVertex3f(0,0,-10)
    glEnd()
    glDisable(GL_LINE_STIPPLE)

def draw_strands(s, color):
    # glScaled(2,2,2)
    glLineWidth(4.0)
    for i in range(n_points[s]):
        for j in range(n_points[s]):
            if j >= i:
                break
            else:
                # print i,j
                if connect[s,i,j] == 1:
                    # print "yes", i,j
                    glBegin(GL_LINES)
                    glColor3f(color[0], color[1], color[2])
                    glVertex3f(points[s,i,0],points[s,i,1],points[s,i,2])
                    glVertex3f(points[s,j,0],points[s,j,1],points[s,j,2])
                    glEnd()
                else:
                    pass
                    # print "no ", i,j, connect[s,i,j]
    glLineWidth(1.0)


def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glutWarpPointer(window_width/2, window_height/2);

    glLoadIdentity()

    glRotatef(xrot, 1.0, 0.0, 0.0)
    glRotatef(yrot, 0.0, 1.0, 0.0)
    glTranslated(-xpos, -ypos, -zpos)

    draw_coordinate()
    draw_strands(0, (1.0, 1.0, 1.0))
    draw_strands(1, (1.0, 1.0, 0.0))
    # draw_strands(2, (0.0, 1.0, 1.0))

    glBindTexture(GL_TEXTURE_2D, texture)


    glEnable(GL_TEXTURE_2D)
    glPushMatrix();
    # glTranslatef(0.0,-2.0,0.0)
    # glRotatef(30.0,0.0,1.0,0.0)
    for numz in range(0,10):
        for num in range(0,10):
            glBindTexture(GL_TEXTURE_2D, texture)   # 2d texture (x and y size)
            glBegin(GL_QUADS);                      # floor
            glTexCoord2f(1.0, 1.0); glVertex3f(-1.0 + num*2, -1.0, -1.0 - numz*2);
            glTexCoord2f(0.0, 1.0); glVertex3f( 1.0 + num*2, -1.0, -1.0 - numz*2);
            glTexCoord2f(0.0, 0.0); glVertex3f( 1.0 + num*2, -1.0,  1.0 - numz*2);
            glTexCoord2f(1.0, 0.0); glVertex3f(-1.0 + num*2, -1.0,  1.0 - numz*2);
            glEnd();
    glPopMatrix()
    glDisable(GL_TEXTURE_2D)

    glPushMatrix();
    glTranslatef(0.0,3.0,-20.0)
    glRotatef(30.0,0.0,1.0,0.0)
    glutSolidCube(2)
    glPopMatrix()

    glutSwapBuffers()
    glutPostRedisplay()


def reshape(w, h):
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(w)/float(h), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def passive_motion(x, y):
    global lastx, lasty, xrot, yrot
    diffx = x - window_width /2 # check the difference between the current x and the last x position
    diffy = y - window_height/2 # check the difference between the current y and the last y position
    # lastx = x         # set lastx to the current x position
    # lasty = y         # set lasty to the current y position
    xrot += 0.2*diffy     # set the xrot to xrot with the addition of the difference in the y position
    yrot += 0.2*diffx     # set the xrot to yrot with the addition of the difference in the x position
    # print "xrot, yrot", xrot, yrot

def mouse(button, state, x, y):
    pass

def keyboard(key, x, y):
    global xpos, ypos, zpos, xrot, yrot, zrot
    move_speed = 0.3
    # print key, x, y
    # global eye
    # diff = center - eye
    navigation_speed = 0.1

    if key == chr(27):
        sys.exit(0)

    elif key == 'e':
        yrotrad = (yrot / 180 * np.pi)
        xpos += float(np.cos(yrotrad)) * 0.2
        zpos += float(np.sin(yrotrad)) * 0.2
    elif key == 'a':
        yrotrad = (yrot / 180 * np.pi)
        xpos -= float(np.cos(yrotrad)) * 0.2
        zpos -= float(np.sin(yrotrad)) * 0.2
    elif key == 'l':
        xrotrad = (xrot / 180 * np.pi)
        yrotrad = (yrot / 180 * np.pi)
        pitchFactor = np.cos(xrotrad)
        yawFactor   = np.cos(xrotrad)
        xpos += move_speed * float(np.sin(yrotrad)) *  1.0 * pitchFactor
        ypos += move_speed * float(np.sin(xrotrad)) * -1.0
        zpos += move_speed * float(np.cos(yrotrad)) * -1.0 * yawFactor;
    elif key == 'r':
        xrotrad = (xrot / 180 * np.pi)
        yrotrad = (yrot / 180 * np.pi)
        pitchFactor = np.cos(xrotrad)
        yawFactor   = np.cos(xrotrad)
        xpos += move_speed * float(np.sin(yrotrad)) * -1.0 * pitchFactor
        ypos += move_speed * float(np.sin(xrotrad)) *  1.0
        zpos += move_speed * float(np.cos(yrotrad)) *  1.0 * yawFactor;

def loadTexture ( fileName, texture ):
    image  = Image.open(fileName)
    width  = image.size[0]
    height = image.size[1]
    image  = image.tostring ( "raw", "RGBX", 0, -1 )

    #    texture = glGenTextures ( 1 )
    glBindTexture     ( GL_TEXTURE_2D, texture )
    glPixelStorei     ( GL_UNPACK_ALIGNMENT,1 )
    glTexParameterf   ( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT )
    glTexParameterf   ( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT )
    glTexParameteri   ( GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR )
    glTexParameteri   ( GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR )
    gluBuild2DMipmaps ( GL_TEXTURE_2D, 3, width, height, GL_RGBA, GL_UNSIGNED_BYTE, image )

    return texture

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glutInitWindowSize(window_width, window_height);
    glutInitWindowPosition(200,200)
    glutCreateWindow("Gauss Linking Integral")
    loadTexture("tex.jpg", texture)
    loadTexture("wall.jpg", textureWall)
    # callbacks setup
    glutWarpPointer(window_width/2, window_height/2);
    glutDisplayFunc(display)
    # glutIdleFunc(display)
    glutReshapeFunc(reshape)
    glutMouseFunc(mouse)
    glutKeyboardFunc(keyboard)
    glutPassiveMotionFunc(passive_motion)
    initGL(window_width, window_height)
    glutMainLoop()

if __name__ == "__main__":
    main()
