#ifndef GUI_H
#define GUI_H
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cairo.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <array>
#include <map>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <string>

//Definitions
#define WINDOW_WIDTH 400
#define WINDOW_HEIGHT 400
#define BUTTON_WIDTH 100
#define BUTTON_HEIGHT 100
#define DEFAULT_LINEAR 2
#define DEFAULT_ANGULAR 2.5
#define LINEAR_MAX 4.0
#define ANGULAR_MAX 5.0
#define SCALE_INC .1
#define VID_DEF_WIDTH 720
#define VID_DEF_HEIGHT 480

//Global Variables
enum Keys {q, w, e, a, s, d};
guint id;
guint id2;
bool repeat;
bool repeat2;
bool currentlyDrawing;
int init = 0;
double linear_speed = DEFAULT_LINEAR;
double angular_speed = DEFAULT_ANGULAR;
double x = 0;
double th = 0;
geometry_msgs::Twist twist;
ros::Publisher move_pub;
GtkWidget *move_window = NULL;
GtkWidget *image_window = NULL;
GtkApplication *app;
GtkWidget *move_menu_button;
GtkWidget *image_menu_button;
cv::Mat input;
cv::Mat output;
cairo_surface_t *surface;
cairo_t *cr;
GdkPixbuf *pixbuf;
GtkWidget *darea;
const std::string TOPIC_NAME = "/kinect2/hd/image_color/";

//map storing grid formats/spacing
std::map<int, std::vector<int>> formats {
  {0, {0, 0, 1, 1}},
  {1, {1, 0, 1, 1}},
  {2, {2, 0, 1, 1}},
  {3, {0, 1, 1, 1}},
  {4, {1, 1, 1, 1}},
  {5, {2, 1, 1, 1}},
  {6, {0, 3, 1, 1}},
  {7, {0, 4, 1, 1}},
  {8, {2, 3, 1, 1}},
  {9, {2, 4, 1, 1}}
};

//map storing movement binding for specific keys
std::map<Keys, std::vector<double>> moveBindings
{
  {w, {1, 0}},
  {e, {1, -1}},
  {a, {0, 1}},
  {d, {0, -1}},
  {q, {1, 1}},
  {s, {-1, 0}}
};


//function declarations
static void init_move();
static void init_image();
static void linear_moved (GtkRange *range, gpointer  user_data);
static void angular_moved (GtkRange *range, gpointer  user_data);
static int movement(gpointer data);
static void pressed (GtkWidget *widget, gpointer data);
static void move_pressed(GtkWidget *widget, gpointer data);
static void vid_toggled(GtkToggleButton *source, gpointer data);
static void released (GtkWidget *widget, gpointer data);
static void init_button(GtkWidget *button, GtkWidget *grid);
static void init_slider(GtkWidget *scale, GtkWidget *label, GtkWidget *grid);
static void activate (GtkApplication *app, gpointer user_data);
gboolean on_key_press (GtkWidget *widget, GdkEventKey *event, gpointer user_data);
gboolean on_widget_deleted(GtkWidget *widget, GdkEvent *event, gpointer data);
gboolean exit_program(GtkWidget *widget, GdkEvent *event, gpointer data);
gboolean draw_callback (GtkWidget *widget, cairo_t *cr, gpointer data);

#endif