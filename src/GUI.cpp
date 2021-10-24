#include <ROS_GUI/GUI.h>


/* 
 * Further todo is to move this to its own .h and .cpp and just link 
 */
class ImageConverter
{
  ros::NodeHandle nh2;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  // create options for subscriber and pass pointer to our custom queue
public:
  ImageConverter() : it_(nh2)
  {
    // Subscrive to input video feed and publish output video feed*/
    image_sub_ = it_.subscribe(TOPIC_NAME, 1000,
      &ImageConverter::imageCb, this);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Update GUI Window
    input = cv_ptr->image;
    //convert and resize image.
    cv::cvtColor (input, input, CV_BGR2RGB);
    resize(input, output, cv::Size(VID_DEF_WIDTH, VID_DEF_HEIGHT), 0, 0, cv::INTER_LINEAR);
    //update drawing
    gtk_widget_queue_draw(image_window);

    /* No clue whats going on here but program breaks without this. I'm assuming that without this running
     * The program gets stuck. hitting ctrl c in terminal will break the image processing but then pop up
     * with the menu. Potential fix by implementing threading? 
     */
    cv::imshow("Whee", output);
    cv::waitKey(1);
    cv::destroyAllWindows();
  
  }
};

/*
 * callback for drawing area, when drawing area must be updated grabs
 * current img data from subscriber and draws it on the drawing area. 
 */
gboolean draw_callback (GtkWidget *widget, cairo_t *cr, gpointer data)
{
  //create pixelbuf from OpenCV mat
    pixbuf = gdk_pixbuf_new_from_data(
        (guint8*)output.data,
        GDK_COLORSPACE_RGB,
        false,
        8,
        output.cols,
        output.rows,
        (int)output.step, NULL, NULL);
    gdk_cairo_set_source_pixbuf(cr, pixbuf, 0, 0);
    //gtk_widget_queue_draw_area (image_window, 0, 0, VID_DEF_WIDTH, VID_DEF_HEIGHT);
    cairo_paint(cr);
    return FALSE;
}

/*  
 * Callback for key press events, check if Ctrl+C is being pressed, if 
 * so then exits program entirely. Lines up with keys on move buttons
 */
gboolean on_key_press (GtkWidget *widget, GdkEventKey *event, gpointer user_data)
{

  switch(event->keyval) {
    case GDK_KEY_c:
      if (event->state & GDK_CONTROL_MASK) {
        exit(0);
      }
      break;
    case GDK_KEY_q:
      movement(GINT_TO_POINTER(0));
      break;
    case GDK_KEY_w:
      movement(GINT_TO_POINTER(1));
      break;
    case GDK_KEY_e:
      movement(GINT_TO_POINTER(2));
      break;
    case GDK_KEY_a:
      movement(GINT_TO_POINTER(3));
      break;
    case GDK_KEY_s:
      movement(GINT_TO_POINTER(4));
      break;
    case GDK_KEY_d:
      movement(GINT_TO_POINTER(5));
      break;
  }
  return FALSE; 
}

/*
 * On widget destroy event, hides the window instead of deleting as to preserve
 * the data,
 */
gboolean on_widget_deleted(GtkWidget *widget, GdkEvent *event, gpointer data)
{
    gtk_widget_hide(widget);
    if(GPOINTER_TO_INT(data) == 1) {
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(image_menu_button), false);
    }

    return TRUE;
}

/* 
 * Callback function for main application window being closed, 
 * exits the program.
 */
gboolean exit_program(GtkWidget *widget, GdkEvent *event, gpointer data)
{
    //if menu closed exit program entirely. 
    exit(0);
    return TRUE;
}

/*
 * Function that initializes all the elements of the movement window
 */
static void init_move() {
  GtkWidget *button;
  GtkWidget *move_speed;
  GtkWidget *turn_speed;
  GtkWidget *move_label;
  GtkWidget *turn_label;
  GtkWidget *grid;

  GtkAdjustment *hadjustment;
  GtkAdjustment *hadjustment_2;

  //setting values for the window
  move_window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (move_window), "Movement");
  gtk_window_set_default_size (GTK_WINDOW (move_window), 400, 300);
  gtk_container_set_border_width(GTK_CONTAINER(move_window), 15);

  //connecting signals for movement and exiting the program. 
  g_signal_connect(move_window, "delete-event", G_CALLBACK(on_widget_deleted), NULL);
  g_signal_connect (G_OBJECT (move_window), "key_press_event", G_CALLBACK (on_key_press), NULL);
  g_signal_connect (G_OBJECT (move_window), "key_release_event", G_CALLBACK (released), GINT_TO_POINTER(1));

  //create grid for GUI element layouts
  grid = gtk_grid_new();
  gtk_grid_set_column_spacing (GTK_GRID (grid), 10);
  gtk_grid_set_column_homogeneous (GTK_GRID (grid), TRUE);
  gtk_container_add (GTK_CONTAINER (move_window), grid);

  //creating and initializing buttons
  button = gtk_button_new_with_label("Forward Left\n           (Q)");
  init_button(button, grid);
  button = gtk_button_new_with_label ("Forward\n     (W)");
  init_button(button, grid);
  button = gtk_button_new_with_label ("Forward Right\n            (E)");
  init_button(button, grid);
  button = gtk_button_new_with_label ("Turn Left\n       (A)");
  init_button(button, grid);
  button = gtk_button_new_with_label ("Backward\n        (S)");
  init_button(button, grid);
  button = gtk_button_new_with_label ("Turn Right\n        (D)");
  init_button(button, grid);

  //initialize the basic pieces of the slider
  move_label = gtk_label_new ("Move to adjust linear speed...");
  turn_label = gtk_label_new ("Move to adjust turn speed...");
  hadjustment = gtk_adjustment_new (DEFAULT_LINEAR, 0, LINEAR_MAX, SCALE_INC, SCALE_INC, 0);
  hadjustment_2 = gtk_adjustment_new (DEFAULT_ANGULAR, 0, ANGULAR_MAX, SCALE_INC, SCALE_INC, 0); 
  move_speed = gtk_scale_new (GTK_ORIENTATION_HORIZONTAL,hadjustment);
  turn_speed = gtk_scale_new (GTK_ORIENTATION_HORIZONTAL, hadjustment_2);

  init_slider(move_speed, move_label, grid);
  init_slider(turn_speed, turn_label, grid);

}

/*  
 * Function to initialize the elements of the image window.
 */
static void init_image() {
  
  //initialize window properties
  image_window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (image_window), "Image Window");
  gtk_window_set_default_size (GTK_WINDOW (image_window), VID_DEF_WIDTH, VID_DEF_HEIGHT);

  //intialize the drawing area.
  darea = gtk_drawing_area_new();
  gtk_container_add(GTK_CONTAINER (image_window), darea);

  //connect the signals to the keys for movement, closing the window, and drawing the image
  g_signal_connect(image_window, "delete-event", G_CALLBACK(on_widget_deleted), GINT_TO_POINTER(1));
  g_signal_connect (G_OBJECT (image_window), "key_press_event", G_CALLBACK (on_key_press), NULL);
  g_signal_connect (G_OBJECT (image_window), "key_release_event", G_CALLBACK (released), GINT_TO_POINTER(1));
  g_signal_connect (G_OBJECT (darea), "draw", G_CALLBACK (draw_callback), NULL);
}

/*
 * Callback function for linear speed slider, adjust global linear speed
 * variable depending on value of slider.
 */
static void linear_moved (GtkRange *range, gpointer  user_data) {

  GtkWidget *label = (GtkWidget *)user_data;
  //Update move speed based on slider value
  linear_speed = gtk_range_get_value (range);
  //update label depending on slider value.
  gchar *str = g_strdup_printf ("Linear Speed: %.2f", linear_speed);
  gtk_label_set_text (GTK_LABEL (label), str);
  g_free(str);
}

/*
 * Callback function for angular speed slider, adjust global angular speed
 * variable depending on value of slider.
 */
static void angular_moved (GtkRange *range, gpointer  user_data) {
  GtkWidget *label = (GtkWidget *)user_data;
  //Update angular speed based on slider value
  angular_speed = gtk_range_get_value (range);
  //update label depending on slider value.
  gchar *str = g_strdup_printf ("Angular Speed: %.2f", angular_speed);
  gtk_label_set_text (GTK_LABEL (label), str);
  g_free (str);
}

/* Callback function tied to timeout created when movement button held
 * Grabs moveBinding based on which button was pressed and then 
 * updates move variables based on bindings. publishes new pose.
 */
static int movement(gpointer data) {
  //get value of key pressed and corresponding move bindings.
  Keys key = (Keys)GPOINTER_TO_INT(data);
  x = moveBindings[key][0];
  th = moveBindings[key][1];
  //update twist msg based on moveBindings and publish new pose.
  twist.linear.x = x*linear_speed;
  twist.angular.z = th*angular_speed;
  move_pub.publish(twist);
  ros::spin();
  return repeat;
}

/*
 * Callback function for movement buttons, repeatedly calls the "movement" function
 * at the interval of every "timeout" milliseconds by creating a timeout, starts looping 
 */
static void pressed(GtkWidget *widget, gpointer data)
{   
  repeat = true;
  int timeout = 0;
  id = g_timeout_add(timeout, (GSourceFunc)movement, data);
}

/*
 * Callback function for move menu button, reshows the
 * movement window if not already displayed.
 */
static void move_pressed(GtkWidget *widget, gpointer data)
{   
  gtk_widget_show_all(move_window);
}

/*
 *  Handles toggle button for image window, if not active shows the image
 *  window otherwise it is untoggle and hides the window.
 */
static void vid_toggled(GtkToggleButton *source, gpointer data)
{
  if(gtk_toggle_button_get_active(source)) {
    gtk_widget_show_all(image_window);
  }
  else {
    gtk_widget_hide(image_window);
  }
}

/*
 * Callback function for releasing movement buttons, removes the timeout/loop, 
 * resets/clears the movement values, and publishes a zeroed out pose, 
 */
static void released (GtkWidget *widget, gpointer data)
{
  //remove timeout and then zero out move variables
  if(GPOINTER_TO_INT(data) == 0) {
    g_source_remove(id);
  }
  x = 0;
  th = 0;
  //zeroing out/updating pose and then publishing it.
  twist.linear.x = x*linear_speed;
  twist.angular.z = th*angular_speed;
  move_pub.publish(twist);
  ros::spin();
  //end the looping
  repeat = false;
}

/* 
 * Function that connects the signals to the buttons
 * and puts them in their proper places on the grid. 
 */
static void init_button(GtkWidget *button, GtkWidget *grid) {
  //connects the held and released signals to the current button
  g_signal_connect (button, "pressed", G_CALLBACK(pressed), GINT_TO_POINTER(init));
  g_signal_connect (button, "released", G_CALLBACK(released), NULL);
  //based on value of current button, grab its format and put it in 
  //right spot on grid
  std::vector<int> current = formats[init];
  gtk_grid_attach(GTK_GRID(grid), button, current[0], current[1], current[2], current[3]);
  gtk_widget_set_size_request(button, BUTTON_WIDTH, BUTTON_HEIGHT);
  //update initialization counter
  init++;
}

/* 
 * Function that connects the slider callbacks to the sliders
 * and puts them in their proper places on the grid. 
 */
static void init_slider(GtkWidget *scale, GtkWidget *label, GtkWidget *grid) {
  //Show two decimal places.
  gtk_scale_set_digits (GTK_SCALE(scale), 2); 
  //setting alignments
  gtk_widget_set_valign (scale, GTK_ALIGN_CENTER);
  gtk_widget_set_valign (label, GTK_ALIGN_START);
  //connecting the two callback functions to the corresponding sliders.
  switch (init) {
    case 6:
    g_signal_connect (scale, "value-changed", G_CALLBACK (linear_moved), label);
    break;
    case 8:
    g_signal_connect (scale, "value-changed", G_CALLBACK (angular_moved), label);
    break;
    default:
    g_print("Error, invalid slider initialization\n");
    break;
  }
  //format based on init counter.
  std::vector<int> current = formats[init];
  gtk_grid_attach(GTK_GRID(grid), scale, current[0], current[1], current[2], current[3]);
  init++;
  current = formats[init];
  gtk_grid_attach(GTK_GRID(grid), label, current[0], current[1], current[2], current[3]);
  init++;
}


/*
 * Setup for the gtk application, calls all the helper methods.
 *
 */
static void activate (GtkApplication *app, gpointer user_data)
{
  GtkWidget *window;
  GtkWidget *button;
  GtkWidget *grid;

  //setting values for the window
  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "ROS Basic GUI");
  gtk_window_set_default_size (GTK_WINDOW (window), 400, 230);
  gtk_container_set_border_width(GTK_CONTAINER(window), 15);

  //create grid for GUI element layouts
  grid = gtk_grid_new();
  gtk_grid_set_column_spacing (GTK_GRID (grid), 10);
  gtk_grid_set_column_homogeneous (GTK_GRID (grid), TRUE);
  gtk_container_add (GTK_CONTAINER (window), grid);


  //setting up move button for the initial menu
  move_menu_button = gtk_button_new_with_label("Movement Menu");
  g_signal_connect (move_menu_button, "clicked", G_CALLBACK(move_pressed), NULL);
  gtk_grid_attach(GTK_GRID(grid), move_menu_button, 0, 0, 1, 1);
  gtk_widget_set_size_request(move_menu_button, BUTTON_WIDTH, BUTTON_HEIGHT);

  //image button setup
  image_menu_button = gtk_toggle_button_new_with_label("Toggle Video");
  g_signal_connect (GTK_TOGGLE_BUTTON(image_menu_button), "toggled", G_CALLBACK(vid_toggled), NULL);
  gtk_grid_attach(GTK_GRID(grid), image_menu_button, 0, 1, 1, 1);
  gtk_widget_set_size_request(image_menu_button, BUTTON_WIDTH, BUTTON_HEIGHT);
  
  init_move();
  init_image();
  
  g_signal_connect (G_OBJECT (window), "key_press_event", G_CALLBACK (on_key_press), NULL);
  g_signal_connect (G_OBJECT (window), "key_release_event", G_CALLBACK (released), GINT_TO_POINTER(1));

  g_signal_connect(window, "destroy", G_CALLBACK(exit_program), NULL);
  gtk_widget_show_all (window);
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "GUI");
  ros::NodeHandle nh;
  move_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist twist;

  surface = cairo_image_surface_create(CAIRO_FORMAT_RGB24, VID_DEF_WIDTH, VID_DEF_HEIGHT);
  cr = cairo_create(surface);

  ImageConverter ic;
  int status;
  
  app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);
  status = g_application_run (G_APPLICATION (app), argc, argv);
  g_object_unref (app);
  return status;
}