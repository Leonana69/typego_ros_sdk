#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <set>
#include <map>
#include <cctype>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joy.hpp>

using namespace std;
using namespace std::chrono;

const double PI = 3.1415926;

double maxYawRate = 80.0;
double maxSpeed = 1.0;

float joyFwd = 0;
float joyLeft = 0;
float joyYaw = 0;

bool running = true;
mutex inputMutex;

// Track which keys are currently pressed (simple set - key is either pressed or not)
set<char> pressedKeys;

// Configure terminal for raw input
void configureTerminal(bool enable) {
  static struct termios oldt, newt;
  static bool configured = false;
  
  if (!isatty(STDIN_FILENO)) {
    return;
  }
  
  if (enable && !configured) {
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;  // Non-blocking
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    configured = true;
  } else if (!enable && configured) {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    configured = false;
  }
}

// Non-blocking character read
int getch_nonblock() {
  if (!isatty(STDIN_FILENO)) {
    return EOF;
  }
  
  fd_set readfds;
  struct timeval timeout;
  
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);
  
  timeout.tv_sec = 0;
  timeout.tv_usec = 10000; // 10ms timeout for better responsiveness
  
  int result = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
  
  if (result > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
    return getchar();
  }
  
  return EOF;
}

// Process all currently pressed keys and update movement values
void updateMovementFromKeys() {
  // Reset all values first
  joyFwd = 0;
  joyLeft = 0;
  joyYaw = 0;
  
  // Apply values based on currently pressed keys
  if (pressedKeys.count('w') > 0) joyFwd = 1.0;
  if (pressedKeys.count('s') > 0) joyFwd = -1.0;
  
  if (pressedKeys.count('a') > 0) joyLeft = 1.0;
  if (pressedKeys.count('d') > 0) joyLeft = -1.0;
  
  if (pressedKeys.count('q') > 0) joyYaw = 1.0;
  if (pressedKeys.count('e') > 0) joyYaw = -1.0;
}

// Keyboard input thread function
void keyboardInputThread() {
  if (!isatty(STDIN_FILENO)) {
    cerr << "ERROR: stdin is not a terminal. Keyboard input will not work." << endl;
    cerr << "Please run this node directly (not via launch file) or ensure stdin is connected." << endl;
    return;
  }
  
  configureTerminal(true);
  
  cout << "Keyboard control active. Use WASD for movement, QE for rotation. Press 'x' to stop." << endl;
  cout << "W/S: Forward/Backward" << endl;
  cout << "A/D: Left/Right" << endl;
  cout << "Q/E: Rotate Left/Right" << endl;
  cout << "Hold keys to move. Release to stop that direction." << endl;
  cout.flush();
  
  // For detecting key releases, we need to track key repeat timing
  map<char, steady_clock::time_point> lastKeyTime;
  const int KEY_REPEAT_THRESHOLD_MS = 100; // If no repeat after this, key was released
  
  while (running) {
    int key = getch_nonblock();
    auto currentTime = steady_clock::now();
    
    lock_guard<mutex> lock(inputMutex);
    
    if (key != EOF) {
      char keyLower = tolower(key);
      
      // Handle key press
      if (keyLower == 'w' || keyLower == 's' || keyLower == 'a' || 
          keyLower == 'd' || keyLower == 'q' || keyLower == 'e') {
        pressedKeys.insert(keyLower);
        lastKeyTime[keyLower] = currentTime;
      } else if (keyLower == 'x') {
        // Stop all movement
        pressedKeys.clear();
        lastKeyTime.clear();
      }
    }
    
    // Check for key releases (no repeat within threshold)
    auto it = lastKeyTime.begin();
    while (it != lastKeyTime.end()) {
      auto timeSinceKey = duration_cast<milliseconds>(currentTime - it->second).count();
      if (timeSinceKey > KEY_REPEAT_THRESHOLD_MS) {
        // Key was released - remove from pressed keys
        pressedKeys.erase(it->first);
        it = lastKeyTime.erase(it);
      } else {
        ++it;
      }
    }
    
    // Update movement values based on current key state
    updateMovementFromKeys();
    
    // Small sleep to prevent CPU spinning
    this_thread::sleep_for(milliseconds(10));
  }
  
  configureTerminal(false);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("teleopKeyboardController");

  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);

  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);

  // Start keyboard input thread
  thread keyboardThread(keyboardInputThread);

  auto pubJoy = nh->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);
  
  rclcpp::Rate rate(50);
  bool status = rclcpp::ok();
  bool wasPublishing = false; // Track if we were publishing in the previous iteration
  
  while (status) {
    rclcpp::spin_some(nh);
    
    bool shouldPublish = false;
    sensor_msgs::msg::Joy joy_msg;
    
    // Read keyboard input values (protected by mutex)
    {
      lock_guard<mutex> lock(inputMutex);
      
      // Publish if keys are pressed, or if we need to send a final stop message
      shouldPublish = !pressedKeys.empty() || wasPublishing;
      
      if (shouldPublish) {
        joy_msg.header.stamp = nh->now();
        joy_msg.header.frame_id = "keyboard";

        joy_msg.axes.push_back(0);
        joy_msg.axes.push_back(0);
        joy_msg.axes.push_back(1.0);
        joy_msg.axes.push_back(joyYaw);
        joy_msg.axes.push_back(joyFwd);
        joy_msg.axes.push_back(1.0);
        joy_msg.axes.push_back(0);
        joy_msg.axes.push_back(0);

        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(1);
        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(0);
        joy_msg.buttons.push_back(0);
      }
      
      // Update state for next iteration
      wasPublishing = !pressedKeys.empty();
    }

    // Only publish if needed
    if (shouldPublish) {
      pubJoy->publish(joy_msg);
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  // Cleanup
  running = false;
  if (keyboardThread.joinable()) {
    keyboardThread.join();
  }

  return 0;
}
