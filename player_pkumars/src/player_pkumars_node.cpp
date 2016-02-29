#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h> 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


using namespace std;

// class team should contain a list of players.
namespace rws2016_pkumars // rws2016 classes are grouped here using namespace
{

class Player
{
	public:
	//Constructor with the same name as the class
	Player (string name_coming){this ->name=name_coming;} // this existing name is equal to the coming variable name from main function
	
	int setTeamName(int team_index = 0 /*default value*/)
	{
		if (team_index==0){setTeamName("red"); return 1;}
		else if (team_index==1){setTeamName("green"); return 1;}
		else if (team_index==2){setTeamName("blue"); return 1;}
		else {cout<<"Not a valis team Index"<<endl; return 0;}
	} // this integer type function uses string type function that exists just below if this 
	
	//Set team name, if given a correct team name (accessor)
	int setTeamName(string team)
	{
		if ((team=="red")||(team=="green")||(team=="blue"))
		{
			this ->team=team; // coming team from main function is assigned to this existing variable "team". 
			return 1;
		}
		else
		{
			cout<<"This is not a valid team"<<endl;
			return 0;
		}
	
	}
	
	//A public atribute
	string name; // existing or declared name
	
	//Gets team name and returns it(accessor) (because this team variable is private and cannot be used outside this class.)
	string getTeamName(void)
	{
		return team;
	}
	
	tf::Transform getPose(void)
	{
		ros::Duration(0.1).sleep(); //To allow the listener to hear messages
		tf::StampedTransform st; //The pose of the player
		static tf::TransformBroadcaster br;
		
		try
		{
			listener.lookupTransform("/map", name, ros::Time(0), st);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		tf::Transform t;
		
		t.setOrigin(st.getOrigin());
		t.setRotation(st.getRotation());
		
		
		return t;
	}
	
	
	private:
	string team;
	tf::TransformListener listener; //gets transforms from the system

};

//Class myPlayer extends class Player (inheritance)
/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */
class MyPlayer: public Player
{
    public: 

		/**
		 * @brief The transform publisher object
		 */
		tf::TransformBroadcaster br;

		/**
		 * @brief Constructor
		 *
		 * @param name player name
		 * @param team team name
		 */
	
    MyPlayer(string name, string team): Player(name) // coming name assigned here 
    {
        setTeamName(team);

		//Initialize position to 0,0,0
		tf::Transform t;
		t.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		tf::Quaternion q; q.setRPY(0, 0, 0);
		t.setRotation(q);
		br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));
    }
    
    /**
	 * @brief Moves MyPlayer
	 *
	 * @param displacement the liner movement of the player, bounded by [-0.1, 1]
	 * @param turn_angle the turn angle of the player, bounded by  [-M_PI/60, M_PI/60]
	 */
	void move(double displacement, double turn_angle)
            {
                //Put arguments withing authorized boundaries
                double max_d =  1; 
                displacement = (displacement > max_d ? max_d : displacement); // turnary operator.

                double min_d =  -0.1; 
                displacement = (displacement < min_d ? min_d : displacement);

                double max_t =  (M_PI/60);
                if (turn_angle > max_t)
                    turn_angle = max_t;
                else if (turn_angle < -max_t)
                    turn_angle = -max_t;

                //Compute the new reference frame
                tf::Transform t_mov;
                t_mov.setOrigin( tf::Vector3(displacement , 0, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, turn_angle);
                t_mov.setRotation(q);

/**
 *  we are not publishing map.
 *  we are publishing only transformations 
 * 
 * 
 * */

                tf::Transform t = getPose();
                t = t  * t_mov;

                //Send the new transform to ROS
                br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));
            }
};

/**
 * @brief this Contains a list of all the players on a team
 */

class Team
{
    public: 

    //Team constructor
    Team(string team, vector<string>& player_names)
    {
       name = team; 

       //Cycle all player names, and create a class player for each
       for (size_t i=0; i < player_names.size(); ++i)
       {
           //Why? Copy constructable ...
           boost::shared_ptr<Player> p(new Player(player_names[i]));
           p->setTeamName(name);
           players.push_back(p);
       }

    }

    void printTeamInfo(void)
    {
        cout << "Team " << name << " has the following players:" << endl;

        for (size_t i=0; i < players.size(); ++i)
            cout << players[i]->name << endl;
    }

    /**
	 * @brief The team name
	 */
	string name;

	/**
	 * @brief A list of Players
	 */
	vector<boost::shared_ptr<Player> > players;
};

}// namespace rws2016_pkumars ends here 


/**
 * @brief The main function
 *
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 *
 * @return result
 */


int main(int argc, char **argv)
{

	//initialize ROS stuff
    ros::init(argc, argv, "player_pkumars_node");
    ros::NodeHandle node;

    //Creating an instance of class MyPlayer
    rws2016_pkumars::MyPlayer my_player("pkumars", "red");
    

    //Infinite loop
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        //Test the get pose method                                                                                     
        tf::Transform t = my_player.getPose();
        cout << "x = " << t.getOrigin().x() << " y = " << t.getOrigin().y() << endl;
        
        //Test the move method
        my_player.move(0.1, -M_PI/6);
        

        ros::spinOnce();
        loop_rate.sleep();
    }


return 1;
}

