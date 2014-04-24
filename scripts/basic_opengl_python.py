import sys
import Image
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

window_width  = 1200
window_height = 800
window_center = np.array([float(window_width)/2, float(window_height)/2])

xpos = 0.0; ypos = 2.0; zpos = 0.0; xrot = 0.0; yrot = 0.0; zrot = 0.0
texture  = 0
textureWall = 1

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

    glEnable(GL_TEXTURE_2D)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
    glutSetCursor(GLUT_CURSOR_NONE)

def draw_coordinate():
    glBegin(GL_LINES)
    glColor3f (0.0, 1.0, 0.0)   # Green for x axis
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
    glColor3f (0.0, 1.0, 0.0)   # Green for -x axis
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

def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glutWarpPointer(window_width/2, window_height/2);

    glLoadIdentity()

    glRotatef(xrot, 1.0, 0.0, 0.0)
    glRotatef(yrot, 0.0, 1.0, 0.0)
    glTranslated(-xpos, -ypos, -zpos)

    draw_coordinate()
    glBindTexture(GL_TEXTURE_2D, texture)

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
    diffx = x - window_width/2  # check the difference between the current x and the last x position
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
