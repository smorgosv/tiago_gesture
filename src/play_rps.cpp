// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

// Library for playing sounds
#include <sound_play/sound_play.h>

#include <string>
#include <vector>
#include <map>
#include "gestures.h"

// Sleeps for the set amount of time. Must be called after a sound call or it will not play.
void sleepok(int t, ros::NodeHandle &nh){
   if (nh.ok())
       sleep(t);
 }



std::vector<std::string> plays = { "rock", "paper", "scissors" };

// Chooses a random number between 0 and 2 and returns the string at that index in the plays vector. This is how the TIAGo will choose what to throw against the player

std::string throwRPS() {
   srand(time(0));
   int x = rand() % 3;
   ROS_DEBUG_STREAM("The random number is: " << x);
   return plays[x];
}

// Maps play states to their outcome, either a draw or who wins (tiago or player)

std::string whoWon(std::string tiago, std::string player) {

   static const std::map<std::string, std::string> outcomes  {
     { "rock:rock", "draw" },
     { "rock:paper", "player" },
     { "rock:scissors", "tiago" },
     { "paper:paper", "draw" },
     { "paper:scissors", "player" },
     { "paper:rock", "tiago" },
     { "scissors:scissors", "draw" },
     { "scissors:paper", "tiago" },
     { "scissors:rock", "player" },
   };

   std::string pairing = tiago + ":" + player;
   // Makes the current play state into a single string and looks that combination up in the previously made map to find the outcome of this play.
   return outcomes.at(pairing);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_gesture");

  // Set Info/Debug level for application
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
     ros::console::notifyLoggerLevelsChanged();
  }

  // Ensures the user has given the right amount of inputs
  if ( argc != 2 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun tiago_gesture play_rps <rock|paper|scissors>");
    ROS_INFO(" ");
    ROS_INFO("\twhere the argument is your play");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  // Ensures that what the user has input is a legal play
  std::string player_move = argv[1];
  if (!(player_move == "rock" || player_move == "paper" || player_move == "scissors")) {
    ROS_INFO(" ");
    ROS_INFO("\tERROR: invalid play, valid plays are [rock|paper|scissors]");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  //finds the path to the yaml file that stores all the gesture waypoints and makes it a string
  std::string yaml_name = ros::package::getPath("tiago_gesture") + "/resources/gestures.yaml";
  
  // Creates a new gesture object and initialises it with the previously found yaml file
  Gesture choice(yaml_name);

  // Tiago's move
  std::string tiago_move = throwRPS();

  ROS_INFO_STREAM("Tiago plays " <<  tiago_move << " to your " << player_move);

  choice.setGesture(tiago_move);
  
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Speaks to the user and plays a bell sound to indicate a start to the game
  sound_play::SoundClient sc;
  sleepok(1,nh);
  sc.say("Let's play rock paper scissors");
  sleepok(1,nh);
  sc.play(2);
  sleepok(1,nh);

  // Tiago gestures rock/paper/scissors
  choice.planCurrentGesture();


  // Determines the winner and sets the appropirate verbal message and gesture.
  std::string winner = whoWon(tiago_move, player_move);
  ROS_INFO_STREAM("The winner is " << winner);
  if (winner == "tiago") {
    choice.setGesture("victory");
    sc.say("Yes, I win this round. Shall we play another?");
    sleepok(1,nh);
    ROS_INFO_STREAM ("Play Victory Dance");
  } else if (winner == "player") {
    choice.setGesture("loss");
    sleepok(1,nh);
    ROS_INFO_STREAM ("Play loss slump");
    sc.say("Oh no, I have lost. You win this time. Let us play once more.");
  } else {
    choice.setGesture("tuck");
    ROS_INFO_STREAM ("A draw has been reached. Resetting for replay.");
    sc.say("It is a draw, lets play again");
    sleepok(1,nh);
  }

  // Tiago gestures victory/loss/draw
  choice.planCurrentGesture();

  spinner.stop();

  return EXIT_SUCCESS;
}


