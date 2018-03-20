/*
====================================================================================================
 
_________        .__                 ___________                     __    
\_   ___ \  ____ |  |   ___________  \__    ___/___________    ____ |  | __
/    \  \/ /  _ \|  |  /  _ \_  __ \   |    |  \_  __ \__  \ _/ ___\|  |/ /
\     \___(  <_> )  |_(  <_> )  | \/   |    |   |  | \// __ \\  \___|    < 
 \______  /\____/|____/\____/|__|      |____|   |__|  (____  /\___  >__|_ \
        \/                                                 \/     \/     \/
        
 Kinect Object Detection System
 Developed by Angeliki Beyko, March 2015
 
 ====================================================================================================
 */

/*                                             LIBRARIES
 ==================================================================================================== */
import SimpleOpenNI.*;                                     // Imports the SimpleOpenNI library for Processing, interfaces with kinect

/*                                      VARIABLE INITIALIZATION
 ==================================================================================================== */
SimpleOpenNI kinect;                                       // Variable for capture device
color trackColor;                                          // Variable for the color we are searching for
PImage currentFrame;                                       // Frame (a datatype for storing images...uses pixels[]array)


/*                                     SERIAL & UDP COMM INITIALIZATION
 ==================================================================================================== */
// To be added

/*                                     Y DISTANCE CALCULATION
 ==================================================================================================== */
int y_dist(int distortedY)
{                                                         // Finds the distance of the y coordinate away from m1, in inches
  final float theta = 60.156;                             // Degrees
  final float thetaS1 = 48.112;                           // Degrees
  final float thetaS2 = 12.044;                           // Degrees
  final float m1 = 362;                                   // Pixels
  final float m2 = 242;                                   // Pixels  
  final float kinectHeight = 19.1;                        // Inches
  float distance, nd, newY_output,                        // Initializes variables for next 6 lines
  deg, rad, firstTerm, secondTerm;

  firstTerm = abs(distortedY - m1);                       // Subtract y value for m1 from the object's y value
  secondTerm = thetaS2/120;                               // Constant of degrees per pixels
  deg = thetaS1 + firstTerm * secondTerm;                 // Add angle from base of kinect to m1, to get degrees from m1's y value to object's y value
  rad = radians(deg);                                     // Converts from degrees to radians
  distance = ((tan(rad)) * (kinectHeight));               // Calculates tangent of radians 
  nd = distance - 21.3;                                   // Subtract 21.3 inches, which is the distance from base of kinect to m1

  println("Distance is " + distance);
  println("New Y output " + nd);

  int Y_p = round((nd / 12)*100);                         // Percentage of how far object is from m1, along the Y axis
  return Y_p;
} 

/*                                     X DISTANCE CALCULATION
 ==================================================================================================== */

int x_dist(int distortedX, int distortedY)
{
  int [] x = {0, 205, 451, 409, 243};                      // Coordinates of X1, X2, X3, X4
  int [] y = {0, 364, 367, 240, 242};                      // Coordinates of Y1, Y2, Y3, Y4
  int l = 12;                                              // Length of one side of the perfect square's border in inches
  float rightSide_x_diff = abs(x[4] - x[1]);               // Difference in x values, on right side of board
  float leftSide_x_diff = abs(x[3] - x[2]);                // Difference in x values, on left side of board
  float rightSide_y_diff = abs(y[4] - y[1]);               // Difference in y values, on right side of board
  float leftSide_y_diff = abs(y[3] - y[2]);                // Difference in y values, on left side of board
  
  float rightM = (rightSide_x_diff / rightSide_y_diff);    // Slope for right side of board
  float leftM = (leftSide_x_diff / leftSide_y_diff);       // Slope for left side of board
  
  float left_border = x[3] + (distortedY - y[3])*leftM;    // The slope is positive, because value from X3 to X4 is increasing...based on the difference relative to the corner, not the abs val of the angle, so the left border when distortedY = y[3] should be x[3]
  float right_border = x[4] - (distortedY - y[4])*rightM;  // The slope is negative, because value from X4 to X1 is decreasing

  
  float w = abs(right_border-left_border);                 // width of trapezoid in pixels at distortedY unit
  float p = abs(distortedX-left_border)/ w;                // from closest edge if there was no distortion, inches from edge of h1...this edge must be same for all measurements
  int X_p = round(p*100);                                  // Percentage of how far object is from h1, along the X axis
  return X_p;
}

/*                                      SETUP / SOFTWARE INITIALIZATION
 ==================================================================================================== */
void setup()                 
{                                                         // This function initializes the screen and creates the first image
  size(640, 480);                                         // Size of window you are looking at
  kinect = new SimpleOpenNI(this);                        // New object, simpleopenni is datatype, 'this' is the arg
  kinect.enableRGB();                                     // Enables kinect's RGB camera (the middle lens)
  //trackColor = color (255,203,200);                         // The RGB values we obtained by clicking on the red pixel in separate program
 trackColor = color (255,255,255);
  smooth ();                                              // Improve image quality
  currentFrame = createImage (640,480, RGB);              // Datatype for storing images
}


/*                                          MAIN PROGRAM
 ====================================================================================================
 ==================================================================================================== */
void draw()
{                                                        // This function draws the image to the screen and updates it        
  kinect.update();                                       // Refreshes the scene
  currentFrame.loadPixels();                             // Loads the pixels
  PImage currentFrame = kinect.rgbImage ();              // Stores the image data from RGB lens
  image(currentFrame,0,0);                               // Image in 2D
  int getY;
  int getX;
  float worldRecord = 500;                               // Before we begin searching, the "world record" for closest color is set to a high number that is easy for the first pixel to beat.
  int closestX = 0;                                      // The X coordinate for the pixel whose RGB value is closest to trackColor
  int closestY = 0;                                      // The Y coordinate for the pixel whose RGB value is closest to trackColor
  int sumX = 0;                                          // The sum of the closestX values
  int sumY = 0;                                          // The sum of the closestY values
  int distortedX = 0;                                    // Average of the sumX values             
  int distortedY = 0;                                    // Average of the sumY values
  
  for(int t = 0; t <64; t++)
  {                                                      // Loops 63 times
    for (int x = 160; x < 480; x ++ )                   
    {                                                    // Walks through every pixel between 150 and 259
      for (int y = 180; y < 380; y ++ )                 
      {                                                  // Walks through every pixel between 255 and 390
        int loc = x + y*currentFrame.width;              // What about currentFrame.height?
        color currentColor = currentFrame.pixels[loc];   // Location of the current pixel
        float r1 = red(currentColor);                    // Red value for current pixel
        float g1 = green(currentColor);                  // Green value for current pixel
        float b1 = blue(currentColor);                   // Blue value for current pixel
        float r2 = red(trackColor);                      // Assigns first value of trackColor to be the Red value
        float g2 = green(trackColor);                    // Assigns second value of trackColor to be the Green value
        float b2 = blue(trackColor);                     // Assigns third value of trackColor to be the Blue value
        float d = dist(r1,g1,b1,r2,g2,b2);               // Using euclidean distance to compare colors....We are using the dist( ) function to compare the current color with the color we are tracking.
        float rd = abs(r2-r1);                           // Difference in red value between trackColor and currentColor for that pixel
      
        if ((d<worldRecord) && (rd < 20))                // If dist function result is less than 500 AND if red color difference is less than 20
        {                                                // If current color is more similar to tracked color than closest color, save current location and current difference
          worldRecord = d;                               // Set worldRecord to that distance value
          closestX = x;                                  // Save the X as closestX
          closestY = y;                                  // Save the Y as closestY
        }
      }
    }
      sumX = sumX+closestX;                              // Sum the X's that were saved from the 64 runs
      sumY = sumY+closestY;                              // Sum the Y's that were saved from the 64 runs
  }
    distortedX = sumX/64;                                // Divide the Sum of X's by 64 to get average X
    distortedY = sumY/64;                                // Divide the Sum of Y's by 64 to get average Y
    
 if (worldRecord < 20)                                   // If the distance saved is less than 20, do the following
 {
  fill(trackColor);                                      // Sets the color used the draw shapes
  strokeWeight(3.0);                                     // Thickness of line used to draw the circle, set in pixels
  stroke(0);                                             // Sets the color used to draw lines and borders around shapes
  ellipse(closestX,closestY,20,20);                      // Draws an ellipse (oval) to the screen, parameters are x-coordinate, y-coordinate, width, height...the height and width are equal here, so it is a circle
  println("X_output is " + distortedX + 
  " and Y_output is " + distortedY );
  getY = y_dist(distortedY);
  getX = x_dist(distortedX, distortedY);
  println("Distance to Y is " + getY);
  println("Distance to X is " + getX);
  //exit();                                                // Breaks out of continuous loop invoked by draw()
 }
  
 try 
 {
  Thread.sleep(100);                                     // Delay in ms
 } 
  catch(InterruptedException x)
  {
  }

}
  
/*                                          END OF MAIN PROGRAM
 ====================================================================================================
 ==================================================================================================== */