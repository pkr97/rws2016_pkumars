#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h> 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <rws2016_libs/team_info.h>
#include <rws2016_msgs/GameMove.h>


using namespace std;

// class team should contain a list of players.
namespace rws2016_pkumars // rws2016 classes are grouped here using namespace
{

class Player
{
	public:
	//Constructor with the same name as the class
	Player (string name_coming){this ->name=name_coming;} // this existing name is equal to the coming variable name from main function
	
	int setTeamName(int team_index = 1 /*default value*/)
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
	/**
	void getDistanceToAllHunters(void)
	{
		//for hunter in hunters 

	}
	*/
	double getDistance(Player& p)
	{
		//computing the distance 
		string first_refframe = p.name;
		string second_refframe = name;

		ros::Duration(0.01).sleep(); //To allow the listener to hear messages
		tf::StampedTransform st; //The pose of the player
		try
		{
			listener.lookupTransform(first_refframe, second_refframe, ros::Time(0), st);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(0.1).sleep();
			return 999;
		}

		tf::Transform t;
		t.setOrigin(st.getOrigin());
		t.setRotation(st.getRotation());

		double x = t.getOrigin().x();
		double y = t.getOrigin().y();

		double norm = sqrt(x*x + y*y);
		return norm;

	}
	
	double getHunterDistance(string hunter)
	{
		//computing the distance 
		string first_refframe = hunter;
		string second_refframe = name;

		ros::Duration(0.01).sleep(); //To allow the listener to hear messages
		tf::StampedTransform st; //The pose of the player
		try
		{
			listener.lookupTransform(first_refframe, second_refframe, ros::Time(0), st);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(0.1).sleep();
			return 999;
		}

		tf::Transform t;
		t.setOrigin(st.getOrigin());
		t.setRotation(st.getRotation());

		double x = t.getOrigin().x();
		double y = t.getOrigin().y();

		double norm = sqrt(x*x + y*y);
		return norm;

	}
	
	double getAngle(string player_name)
	{
		//computing the distance 
		string first_refframe = name;
		string second_refframe = player_name;
		
		ros::Duration(0.01).sleep(); //To allow the listener to hear messages
		tf::StampedTransform st; //The pose of the player
		try
		{
			listener.lookupTransform(first_refframe, second_refframe, ros::Time(0), st);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(0.1).sleep();
			return 0;
		}
		
		tf::Transform t;
		t.setOrigin(st.getOrigin());
		t.setRotation(st.getRotation());
		
		double x = t.getOrigin().x();
		double y = t.getOrigin().y();
		
		double angle = atan2(y,x);
		return angle;
	
	}
	
	
	//Gets team name and returns it(accessor) (because this team variable is private and cannot be used outside this class.)
	string getTeamName(void)
	{
		return team;
	}
	
	tf::Transform getPose(void)
	{
		ros::Duration(0.01).sleep(); //To allow the listener to hear messages
        tf::StampedTransform st; //The pose of the player
        try
        {
           listener.lookupTransform("/map", name, ros::Time(0), st);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.1).sleep();
        }

		tf::Transform t;
		t.setOrigin(st.getOrigin());
		t.setRotation(st.getRotation());
		return t;
	}
	
	
	private:
	/**
	 * @brief The name of the team
	 */
	string team;

	/**
	 * @brief the transform listener object
	 */
	tf::TransformListener listener; //reads tfs from the ros system

};


/**
 * @brief this Contains a list of all the players on a team
 */

class Team
{
    public: 
    
    /**
	 * @brief Constructor
	 *
	 * @param team the team name
	 * @param player_names a list with the name of all the players on the team
	 */

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
		
	    //vector<string> prey_names;
		//prey_names.push_back("lalmeida");
		//rws2016_moliveira::Team prey_team("green", prey_names);
		//prey_team.printTeamInfo();

		/**
		* @brief The teams
		*/
		boost::shared_ptr<Team> my_team;
		boost::shared_ptr<Team> hunter_team;
		boost::shared_ptr<Team> prey_team;
		
		boost::shared_ptr<ros::Subscriber> _sub;
	
		/**
		 * @brief Constructor
		 *
		 * @param name player name
		 * @param team team name
		 */
	
	    MyPlayer(string name, string team): Player(name) // coming name assigned here 
	    {
			setTeamName(team);
			ros::NodeHandle node;
			
			//Initialize teams
			vector<string> myTeam_names, myHunters_names, myPreys_names;
			string myTeamId, myHuntersId, myPreysId;
			
			if (!team_info(node, myTeam_names, myHunters_names, myPreys_names, myTeamId, myHuntersId, myPreysId))
			ROS_ERROR("Something went wrong reading teams");
			
			my_team = (boost::shared_ptr<Team>) new Team(myTeamId, myTeam_names);
			hunter_team = (boost::shared_ptr<Team>) new Team(myHuntersId, myHunters_names);
			prey_team = (boost::shared_ptr<Team>) new Team(myPreysId, myPreys_names);
			
			my_team->printTeamInfo();
			hunter_team->printTeamInfo();
			prey_team->printTeamInfo();
			
			//Initialize position according to team
			ros::Duration(0.2).sleep(); //sleep to make sure the time is correct
			tf::Transform t;
			
			struct timeval t1;      
            gettimeofday(&t1, NULL);
            srand(t1.tv_usec);
			
			//srand(time(0)*1000); // To start the player in a random location
			double X=((((double)rand()/(double)RAND_MAX) ) * 2 -1) * 5 ;
			double Y=((((double)rand()/(double)RAND_MAX) ) * 2 -1) * 5 ;
			t.setOrigin( tf::Vector3(X, Y, 0.0) );
			tf::Quaternion q; q.setRPY(0, 0, 0);
			t.setRotation(q);
			br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));
			
			//initialize the subscriber
			_sub = (boost::shared_ptr<ros::Subscriber>) new ros::Subscriber;
			*_sub = node.subscribe("/game_move", 1, &MyPlayer::moveCallback, this);  
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
                displacement = (displacement > max_d ? max_d : displacement);

                double min_d =  -0.1; 
                displacement = (displacement < min_d ? min_d : displacement);

                double max_t =  (M_PI/30);
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

                tf::Transform t = getPose();
                t = t  * t_mov;

                //Send the new transform to ROS
                br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", name));
            }
            
            
       string getNameOfClosestPrey(void)
            {
                double prey_dist = getDistance(*prey_team->players[0]);
                string prey_name = prey_team->players[0]->name;

                for (size_t i = 1; i < prey_team->players.size(); ++i)
                {
                    double d = getDistance(*prey_team->players[i]);

                    if (d < prey_dist) //A new minimum
                    {
                        prey_dist = d;
                        prey_name = prey_team->players[i]->name;
                    }
                }

                return prey_name;
            }
            
            string getNameOfClosestHunter(void)
            {
                double hunter_dist = getDistance(*hunter_team->players[0]);
                string hunter_name = hunter_team->players[0]->name;

                for (size_t i = 1; i < prey_team->players.size(); ++i)
                {
                    double d = getDistance(*hunter_team->players[i]);

                    if (d < hunter_dist) //A new minimum
                    {
                        hunter_dist = d;
                        hunter_name = hunter_team->players[i]->name;
                    }
                }

                return hunter_name;
            }

		/**
             * @brief called whenever a /game_move msg is received
             *
             * @param msg the msg with the animal values
             */
            void moveCallback(const rws2016_msgs::GameMove& msg)
            {
                ROS_INFO("player %s received game_move msg", name.c_str()); 

                //I will encode a very simple hunting behaviour:
                //
                //1. Get closest prey name
                //2. Get angle to closest prey
                //3. Compute maximum displacement
                //4. Move maximum displacement towards angle to prey (limited by min, max)

                //Step 1
                string closest_prey = getNameOfClosestPrey();
                string closest_hunter = getNameOfClosestHunter();
                
                //ROS_INFO("Closest prey is %s", closest_prey.c_str());
                //ROS_INFO("Closest prey is %s", closest_hunter.c_str());

                //Step 2
                double angle_prey = getAngle(closest_prey);
                double angle_hunter = getAngle(closest_prey);

                //Step 3
                double displacement = msg.cat; //I am a cat, others may choose another animal
				double distance_hunter = getHunterDistance(closest_hunter);
				
				
				move(displacement, angle_prey);
				
				double angle;
                //double displacement;

                if (distance_hunter < 2.0 && distance_hunter > 1.5)
                {
                    angle = angle_prey + M_PI/2;
                    displacement = -0.1;
                }

                if (distance_hunter < 1.5)
                {
                    angle = angle_prey + M_PI/2;
                    displacement = msg.cat;
                }
                else if (distance_hunter>2)
                {
                     angle = angle_prey;
                     displacement = msg.cat;
                }
				
				
            }

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
    ros::init(argc, argv, "pkumars");
    ros::NodeHandle node;

    //Creating an instance of class MyPlayer
    rws2016_pkumars::MyPlayer my_player("pkumars", "green");
	
	//Infinite loop
    ros::spin();
    
}

