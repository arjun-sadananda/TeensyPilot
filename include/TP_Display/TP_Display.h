
#include <ILI9341_t3.h>
#include <font_Arial.h>

#include "quaternion.h"
#include "vector3.h"
#include "matrix3.h"

#define PIN_CS 10
#define PIN_DC 9

#include <string.h>

class TP_Display{

private:

    const Rotation NED_TO_DISPLAY = ROTATION_YAW_270;

    uint16_t C_DKBLUE = 0;
    uint16_t C_CYAN = 0;
    ILI9341_t3 Display = ILI9341_t3(PIN_CS, PIN_DC);

    Vector3f vert[8];
    Vector3f face_center[3], rot_face_center[3];
    Vector3f rot_vert[8];
    Vector2f cube_center, rp_center, y_center;
    int rp_size;

    Vector3f ball_center;
    int ball_size;


protected:
public:
    // TP_Display(){
    //     // Display = Display;
    //     // Display.CS
    //     // ILI9341_t3 Display = ILI9341_t3(PIN_CS, PIN_DC);
        
    // }
    void display_setup(){
        
        C_DKBLUE = Display.color565(0, 0, 40);
        C_CYAN = Display.color565(0, 255, 255);

        Display.begin();
        Display.setRotation(3);
        Display.fillScreen(ILI9341_BLACK);

        Display.setFont(Arial_10);
        Display.fillRect(0, 0, 320, 25, C_DKBLUE);
        Display.setTextColor(C_CYAN, ILI9341_BLACK);
        Display.setCursor(10, 7);
        Display.print("TeensyPilot");
    }
    
    void init_cube(){
        vert[0].x = vert[1].x = vert[4].x = vert[5].x = -50.0;
        vert[2].x = vert[3].x = vert[6].x = vert[7].x = 50.0;

        vert[0].y = vert[3].y = vert[4].y = vert[7].y = 50.0;
        vert[1].y = vert[2].y = vert[5].y = vert[6].y = -50.0;

        vert[0].z = vert[1].z = vert[2].z = vert[3].z = -50.0;
        vert[4].z = vert[5].z = vert[6].z = vert[7].z = 50.0;

        face_center[0].x = 50;
        face_center[1].y = -50;
        face_center[2].z = 50;
    }
    void euler_display_setup(){
        display_setup();
        cube_center.x = 220;
        cube_center.y = 125;
        rp_center.x = y_center.x = 70;
        rp_center.y = 90;
        y_center.y = 170;
        rp_size = 80;

        Display.drawRect(rp_center.x-rp_size/2-1, rp_center.y-rp_size/2-1, rp_size+2, rp_size+2, ILI9341_WHITE);
        Display.drawCircle(y_center.x, y_center.y, 32, ILI9341_WHITE);
        //0-1-2-3-0 4-5-6-7-4       0-4 1-5 2-6 3-7
        init_cube();
    }
    void mag_acc_display_setup(){
        display_setup();
        ball_center.x = 160;
        ball_center.y = 135;
        ball_center.z = 0;
        ball_size = 85;

        Display.drawCircle(ball_center.x, ball_center.y, ball_size, ILI9341_WHITE);
    }
    void triad_display_setup(){
        display_setup();
        ball_center.x = 75;
        ball_center.y = 135;
        ball_center.z = 0;
        ball_size = 60;

        Display.drawCircle(ball_center.x, ball_center.y, ball_size, ILI9341_WHITE);
        
        cube_center.x = 220;
        cube_center.y = 125;
        init_cube();
    }
    void ekf_display_setup(){
        // display_setup();
        triad_display_setup();
        // cube_center.x = 220;
        // cube_center.y = 125;
        // init_cube();
    }


    void printTime(int t){//-90deg to 90deg
        Display.setFont(Arial_8);
        Display.setCursor(10, 30);
        Display.fillRect(10, 30, 40, 10, ILI9341_BLACK);
        Display.print(t);
    }
    void printStatus(String string){//-90deg to 90deg
        Display.setFont(Arial_8);
        Display.fillRect(10, 220, 100, 10, ILI9341_BLACK);
        Display.setCursor(10, 220);
        Display.print(string);
    }

    void printVector(Vector3f vector, int x, int y, uint16_t color = ILI9341_CYAN){//-90deg to 90deg
        Display.setFont(Arial_8);
        
        Display.setTextColor(color, ILI9341_BLACK);
        Display.fillRect(x, y, 40, 30, ILI9341_BLACK);
        Display.setCursor(x, y);
        Display.print(vector.x);
        Display.setCursor(x, y+11);
        Display.print(vector.y);
        Display.setCursor(x, y+22);
        Display.print(vector.z);
        
        Display.setTextColor(C_CYAN, ILI9341_BLACK);
    }
    



    void drawNeedle(Vector3f vect, uint16_t color){
        static float s_w, d = 300.0;
        s_w = 275.0/(d-vect.z);
        Display.drawLine(ball_center.x, ball_center.y, ball_center.x+(int)vect.x*s_w, ball_center.y+(int)vect.y*s_w, color);
    }
    // the vectors must be given in NED frame
    void draw_acc_mag_in_ball(Vector3f acc_vect, Vector3f mag_vect){
        static Vector3f old_acc_vect, old_mag_vect;
        acc_vect.rotate(NED_TO_DISPLAY);
        mag_vect.rotate(NED_TO_DISPLAY);

        drawNeedle(old_acc_vect.normalized()*ball_size, ILI9341_BLACK);
        drawNeedle(old_mag_vect.normalized()*ball_size, ILI9341_BLACK);
        if(acc_vect.z>0)
            drawNeedle(acc_vect.normalized()*ball_size, ILI9341_CYAN);
        else
            drawNeedle(acc_vect.normalized()*ball_size, ILI9341_BLUE);
        if(mag_vect.z<0)
            drawNeedle(mag_vect.normalized()*ball_size, ILI9341_ORANGE);
        else
            drawNeedle(mag_vect.normalized()*ball_size, ILI9341_RED);
        old_acc_vect = acc_vect;
        old_mag_vect = mag_vect;
    }

    void draw_acc_mag_in_ball2(Vector3f acc_vect, Vector3f mag_vect){
        static Vector3f old_acc_vect2, old_mag_vect2;
        acc_vect.rotate(NED_TO_DISPLAY);
        mag_vect.rotate(NED_TO_DISPLAY);

        drawNeedle(old_acc_vect2.normalized()*ball_size, ILI9341_BLACK);
        drawNeedle(old_mag_vect2.normalized()*ball_size, ILI9341_BLACK);
        if(acc_vect.z>0)
            drawNeedle(acc_vect.normalized()*ball_size, ILI9341_GREEN);
        else
            drawNeedle(acc_vect.normalized()*ball_size, ILI9341_DARKGREEN);
        if(mag_vect.z<0)
            drawNeedle(mag_vect.normalized()*ball_size, ILI9341_PINK);
        else
            drawNeedle(mag_vect.normalized()*ball_size, ILI9341_MAGENTA);
        old_acc_vect2 = acc_vect;
        old_mag_vect2 = mag_vect;
    }
   

    void draw_euler_deg(float x, float y, float z){//-90deg to 90deg
    
        static float old_x = 0, old_y =0, old_z = 0;
        static int size = 3, r =30;
        x = int(rp_center.x - x*180.0/3.14);
        y = int(rp_center.y - y*180.0/3.14);
        if(x <= rp_center.x-rp_size/2) x = rp_center.x-rp_size/2;
        if(y <= rp_center.y-rp_size/2) y = rp_center.y-rp_size/2;
        
        if(x >= rp_center.x+rp_size/2) x = rp_center.x+rp_size/2-size;
        if(y >= rp_center.y+rp_size/2) y = rp_center.y+rp_size/2-size;
        
        Display.drawCircle(old_x, old_y, size, ILI9341_BLACK);
        Display.drawCircle(x,y,size,ILI9341_GREEN);
        
        Display.drawLine(y_center.x, y_center.y, y_center.x+r*sin(old_z), y_center.y-r*cos(old_z), ILI9341_BLACK);
        Display.drawLine(y_center.x, y_center.y, y_center.x+r*sin(-z), y_center.y-r*cos(-z), ILI9341_GREEN);

        old_x = x;
        old_y = y;
        old_z = -z;
        // Display.drawLine
    }

    void drawEdge(Vector3f v, Vector3f w, uint16_t color){
        static float s_v, s_w, d = 300.0;
        s_v = 275.0/(d+v.z);
        s_w = 275.0/(d+w.z);
        //mirroring y to follow right hand rule NOPE DONT
        Display.drawLine(cube_center.x+(int)v.x*s_v, cube_center.y+(int)v.y*s_v, cube_center.x+(int)w.x*s_w, cube_center.y+(int)w.y*s_w, color);
    }
    void drawCube(float roll, float pitch, float yaw){//-90deg to 90deg  
        static Matrix3f rotation;
        rotation.from_euler(roll, pitch, yaw);
        // rotation.rotate
        drawCube(rotation.transposed()); 
    }
    void drawCube(Quaternion q){//-90deg to 90deg
        static Matrix3f rotation;
        q.rotation_matrix(rotation);
        // rotation.rotate
        drawCube(rotation.transposed()); 
    }
    void drawCube(Matrix3f DCM){//-90deg to 90deg
        static Vector3f center;
        
        static Rotation NED_TO_DISPLAY = ROTATION_YAW_270;

        // erase
        for (int i =0; i<3; i++){
            drawEdge(rot_vert[i], rot_vert[i+1], ILI9341_BLACK);
            drawEdge(rot_vert[4+i], rot_vert[4+i+1], ILI9341_BLACK);
        }
        drawEdge(rot_vert[3], rot_vert[0], ILI9341_BLACK);
        drawEdge(rot_vert[7], rot_vert[4], ILI9341_BLACK);
        for (int i =0; i<4; i++)
            drawEdge(rot_vert[i], rot_vert[i+4], ILI9341_BLACK);
        for (int i =0; i<3; i++)
            drawEdge(center, rot_face_center[i], ILI9341_BLACK);

        // rotate
        for (int i =0; i<8; i++){
            rot_vert[i] = vert[i];
            rot_vert[i].rotate_inverse(NED_TO_DISPLAY);
            rot_vert[i] = DCM*rot_vert[i];
            rot_vert[i].rotate(NED_TO_DISPLAY);
        }
        for (int i =0; i<3; i++){
            rot_face_center[i] = face_center[i];
            rot_face_center[i].rotate_inverse(NED_TO_DISPLAY);
            rot_face_center[i] = DCM*rot_face_center[i];
            rot_face_center[i].rotate(NED_TO_DISPLAY);
        }

        // draw
        for (int i =0; i<3; i++){
            drawEdge(rot_vert[i], rot_vert[i+1], ILI9341_GREEN);
            drawEdge(rot_vert[4+i], rot_vert[4+i+1], ILI9341_RED);
        }
        drawEdge(rot_vert[3], rot_vert[0], ILI9341_GREEN);
        drawEdge(rot_vert[7], rot_vert[4], ILI9341_RED);
        for (int i =1; i<3; i++)
            drawEdge(rot_vert[i], rot_vert[i+4], ILI9341_BLUE);
        for (int i =0; i<4; i+=3)
            drawEdge(rot_vert[i], rot_vert[i+4], ILI9341_CYAN);
        for (int i =0; i<3; i++)
            drawEdge(center, rot_face_center[i], ILI9341_DARKGREY);
    }

};