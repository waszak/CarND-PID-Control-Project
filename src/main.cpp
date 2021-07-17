#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}
double deg2rad(double x)
{
    return x * pi() / 180;
}
double rad2deg(double x)
{
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

void reset(uWS::WebSocket<uWS::SERVER> ws)
{
    string msg = "42[\"reset\",{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void manual(uWS::WebSocket<uWS::SERVER> ws)
{
    string msg = "42[\"manual\",{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void send_position(uWS::WebSocket<uWS::SERVER> ws, double steer_value, double throttle)
{
    json msgJson;
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = throttle;
    string msg = "42[\"steer\"," + msgJson.dump() + "]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    //std::cout << msg << std::endl;
}

struct twiddle_data
{
    int n = 5000;
    int skip = 100;
    // tollerance
    double tol=0.0002;
    // numbers of  iteration.
    int it = 0;
    // dp_idx we improve;
    int dp_idx = 0;

    int total_it = 0;

    bool first_run = true;
    bool increased = true;
    double best_error = 1000;
    double error = 0;
    bool second_time = false;

    double error_per_it = 10;


    std::vector<double> p;
    std::vector<double> best_p;
    std::vector<double> dp;
};



void twiddle(uWS::Hub & h, PID & pid,  twiddle_data &td)
{


    //parameters to optimize
    td.p.push_back(0.000);
    td.p.push_back(0.000);
    td.p.push_back(0.000);
    //delta parameters. How much we gonna change p
    td.dp.push_back(1);//0.06
    td.dp.push_back(1);
    td.dp.push_back(1);
    std::vector<double> best_p;
    td.best_p.push_back(td.p[0] );
    td.best_p.push_back(td.p[1] );
    td.best_p.push_back(td.p[2]);

    pid.Init(td.p[0], td.p[1], td.p[2]);


    h.onMessage([&pid, &td](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                            uWS::OpCode opCode)
    {

        double max_steering = pi()/ 4.0;

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(string(data).substr(0, length));

            if (s != "")
            {

                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {

                    std::cout<<"Parameters: "<<td.p[0]<<" "<<td.p[1]<<" "<<td.p[2]<<"\n"<<"It: "<<td.it<<" Par Idx: "<<td.dp_idx<<" Err: "<<td.error <<"\n";
                    std::cout<<"Best Parameters: "<<td.best_p[0]<<" "<<td.best_p[1]<<" "<<td.best_p[2]<<std::endl;
                    std::cout<<"Sum: "<<td.dp[0]+td.dp[1]+td.dp[2]<<std::endl;
                    if(td.it == 0)
                    {
                        pid.Init(td.p[0], td.p[1], td.p[2]);
                        td.error = 0;
                        td.total_it++;
                        std::cout<<"Total iterations:"<<td.total_it<<"\n";
                        if(!td.first_run)
                        {
                            std::cout<<"Best error:"<<td.best_error<<"\n";
                        }

                        //std::cout<<"Parameters: "<<td.p[0]<<" "<<td.p[1]<<" "<<td.p[2]<<"\n";
                        //std::cout<<"Sum: "<<td.dp[0]+td.dp[1]+td.dp[2]<<std::endl;
                        if(td.dp[0]+td.dp[1]+td.dp[2] <td.tol)
                        {
                            int a;
                            std::cin>>a;
                        }
                    }
                    td.it++;
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value;
                    double throttle = 0.3;

                    pid.UpdateError(cte);
                    //steer value should be between -1 and 1
                    steer_value = pid.TotalError();

                    //we don't calculate error in first n itteration.
                    if(td.it < td.skip)
                    {
                        send_position(ws, steer_value, throttle);
                        return;
                    }
                    //punish high steering value with absolute value higher than 1.
                    //make very small steering value punishment for value bellow 1.
                    //it improves learning speed.
                    //When steer value is small then cte is more important.
                    //We want to minimize cte but early on steering angles are issues.
                    td.error += pow(cte,2)+ pow(steer_value,4);

                    if( td.first_run && (td.it >= td.n))
                    {
                        td.first_run = false;
                        td.best_error = td.error;
                        td.p[0] += td.dp[0];
                        td.it=0;
                        reset(ws);
                        return;
                    }
                    //if error is bigger then we can just stop
                    if (!td.first_run &&( td.it >= td.n || td.error> td.best_error || td.error/td.it > td.error_per_it ))
                    {
                        if( td.error< td.best_error &&  td.error/td.it < td.error_per_it )
                        {
                            td.best_error = td.error;
                            td.dp[td.dp_idx] *= 1.1;
                            td.dp_idx= (td.dp_idx+1)%td.dp.size();
                            td.second_time = false;

                            for(int i =0; i< td.p.size(); i++)
                            {
                                td.best_p[i]=td.p[i];
                            }

                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                        }
                        else if(!td.second_time)
                        {
                            td.p[td.dp_idx] -= 2 * td.dp[td.dp_idx];
                            td.second_time = true;
                        }
                        else
                        {
                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                            td.dp[td.dp_idx] *= 0.9;
                            td.second_time = false;
                            td.dp_idx= (td.dp_idx+1)%td.dp.size();
                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                        }

                        td.it=0;
                        reset(ws);
                        return;
                    }
                    send_position(ws, steer_value, throttle);
                    // DEBUG




                }  // end "telemetry" if
            }
            else
            {
                // Manual driving
                manual(ws);
            }
        }  // end websocket message if
    }); // end h.onMessage
}



void twiddle_throttle(uWS::Hub & h, PID & pid_steering, PID & pid_throttle,  twiddle_data &td)
{

    //0.609181, 0, 0.685517
    //0.6 -1.75075e-05 0.3813
    //parameters to optimize
    td.p.push_back(0.6);
    td.p.push_back(0);
    td.p.push_back(0.4);
    //delta parameters. How much we gonna change p
    td.dp.push_back(0.1);
    td.dp.push_back(0);
    td.dp.push_back(0.1);
    std::vector<double> best_p;
    td.best_p.push_back(td.p[0] );
    td.best_p.push_back(td.p[1] );
    td.best_p.push_back(td.p[2]);

    //there could be also diffrent optimum like 0.00
    pid_steering.Init(0.18502, 0.000324643, 4.57675);
    pid_throttle.Init(td.p[0], td.p[1], td.p[2]);
    td.skip = 0;
    td.n = 2000;

    h.onMessage([&pid_steering, &pid_throttle, &td](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode)
    {

        double max_steering = 1;
        double target_throttle = 0.8;

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(string(data).substr(0, length));

            if (s != "")
            {

                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {


                    if(td.it == 0)
                    {
                        std::cout<<"Parameters: "<<td.p[0]<<" "<<td.p[1]<<" "<<td.p[2]<<"\n"<<"It: "<<td.it<<" Par Idx: "<<td.dp_idx<<" Err: "<<td.error <<"\n";
                        std::cout<<"Best Parameters: "<<td.best_p[0]<<" "<<td.best_p[1]<<" "<<td.best_p[2]<<std::endl;
                        std::cout<<"Sum: "<<td.dp[0]+td.dp[1]+td.dp[2]<<std::endl;
                        std::cout<<"Max iterations: "<<td.n<<std::endl;

                        pid_throttle.Init(td.p[0], td.p[1], td.p[2]);
                        pid_steering.Init(0.18502, 0.000324643, 4.57675);
                        td.error = 0;
                        td.total_it++;
                        std::cout<<"Total iterations:"<<td.total_it<<"\n";
                        if(!td.first_run)
                        {
                            std::cout<<"Best error:"<<td.best_error<<"\n";
                        }

                        //std::cout<<"Parameters: "<<td.p[0]<<" "<<td.p[1]<<" "<<td.p[2]<<"\n";
                        //std::cout<<"Sum: "<<td.dp[0]+td.dp[1]+td.dp[2]<<std::endl;
                        /*if(td.dp[0]+td.dp[1]+td.dp[2] <td.tol)
                        {
                            int a;
                            std::cin>>a;
                        }*/
                    }
                    td.it++;
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value;
                    double throttle;

                    pid_throttle.UpdateError(cte);
                    pid_steering.UpdateError(cte);

                    steer_value = pid_steering.TotalError();

                    if( steer_value > max_steering )
                    {
                        steer_value = max_steering;
                    }
                    else if (steer_value < -max_steering)
                    {
                        steer_value = -max_steering;
                    }

                    throttle = target_throttle + pid_throttle.TotalError();
                    //make sure start is the same
                    if(td.it < 10)
                    {
                        throttle = 1;
                        steer_value = 0;
                    }

                    //we don't calculate error in first n itteration.
                    if(td.it < td.skip)
                    {
                        send_position(ws, steer_value, throttle);
                        return;
                    }

                    td.error += pow(cte, 4);
                    /*if(speed < 20)
                    {
                        td.error += pow(speed, 2);
                    }
                    if(throttle < 0){
                        td.error += pow((1/throttle), 6);
                    }*/
                    //punish throttle higher than 1
                    //and punish close to 0 throttle

                    if( td.first_run && (td.it >= td.n))
                    {
                        td.first_run = false;
                        td.best_error = td.error;
                        td.p[0] += td.dp[0];
                        td.it=0;
                        reset(ws);
                        return;
                    }
                    //increase distance
                    if( td.increased  && !td.first_run && td.error < td.best_error && td.it == td.n  )
                    {
                        td.n +=200;
                        td.increased = true;
                    }

                    //if error is bigger then we can just stop
                    if (!td.first_run &&( td.it >= td.n || td.error> td.best_error))
                    {

                        if( td.error< td.best_error || td.increased )
                        {
                            td.best_error = td.error;
                            td.dp[td.dp_idx] *= 1.1;
                            td.dp_idx= (td.dp_idx+1)%td.dp.size();
                            td.second_time = false;

                            for(int i =0; i< td.p.size(); i++)
                            {
                                td.best_p[i]=td.p[i];
                            }

                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                        }
                        else if(!td.second_time)
                        {
                            td.p[td.dp_idx] -= 2 * td.dp[td.dp_idx];
                            td.second_time = true;
                        }
                        else
                        {
                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                            td.dp[td.dp_idx] *= 0.9;
                            td.second_time = false;
                            td.dp_idx= (td.dp_idx+1)%td.dp.size();
                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                        }
                        td.increased = false;
                        td.it=0;
                        reset(ws);
                        return;
                    }
                    send_position(ws, steer_value, throttle);
                    // DEBUG




                }  // end "telemetry" if
            }
            else
            {
                // Manual driving
                manual(ws);
            }
        }  // end websocket message if
    }); // end h.onMessage
}


void twiddle_steering(uWS::Hub & h, PID & pid_steering, PID & pid_throttle,  twiddle_data &td)
{

    //0.18502 0.000324643 4.57675
    //parameters to optimize
    td.p.push_back(0.18502);
    td.p.push_back(0.000324643);
    td.p.push_back(4.57675);
    //delta parameters. How much we gonna change p
    td.dp.push_back(0.00001);
    td.dp.push_back(0.000001);
    td.dp.push_back(0.00001);
    std::vector<double> best_p;
    td.best_p.push_back(td.p[0] );
    td.best_p.push_back(td.p[1] );
    td.best_p.push_back(td.p[2]);

    //there could be also diffrent optimum like 0.00
    pid_throttle.Init(0.6, -1.75075e-05, 0.3813);
    pid_steering.Init(td.p[0], td.p[1], td.p[2]);
    td.skip = 0;
    td.n = 100;
    td.tol = 0;
    h.onMessage([&pid_steering, &pid_throttle, &td](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode)
    {

        double max_steering = 1;
        double target_throttle = 0.8;

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(string(data).substr(0, length));

            if (s != "")
            {

                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {


                    if(td.it == 0)
                    {
                        std::cout<<"Parameters: "<<td.p[0]<<" "<<td.p[1]<<" "<<td.p[2]<<"\n"<<"It: "<<td.it<<" Par Idx: "<<td.dp_idx<<" Err: "<<td.error <<"\n";
                        std::cout<<"Best Parameters: "<<td.best_p[0]<<" "<<td.best_p[1]<<" "<<td.best_p[2]<<std::endl;
                        std::cout<<"Sum: "<<td.dp[0]+td.dp[1]+td.dp[2]<<std::endl;

                        pid_steering.Init(td.p[0], td.p[1], td.p[2]);
                        pid_throttle.Init(0.6, -1.75075e-05, 0.3813);
                        td.error = 0;
                        td.total_it++;
                        std::cout<<"Total iterations:"<<td.total_it<<"\n";
                        if(!td.first_run)
                        {
                            std::cout<<"Best error:"<<td.best_error<<"\n";
                        }

                        //std::cout<<"Parameters: "<<td.p[0]<<" "<<td.p[1]<<" "<<td.p[2]<<"\n";
                        //std::cout<<"Sum: "<<td.dp[0]+td.dp[1]+td.dp[2]<<std::endl;
                        if(td.dp[0]+td.dp[1]+td.dp[2] <td.tol)
                        {
                            int a;
                            std::cin>>a;
                        }
                    }
                    td.it++;
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value;
                    double throttle;

                    pid_throttle.UpdateError(cte);
                    pid_steering.UpdateError(cte);

                    steer_value = pid_steering.TotalError();


                    /*if(fabs(steer_value)> 1)
                    {
                       td.error += pow(steer_value,2);

                    }*/

                    if( steer_value > max_steering )
                    {
                        steer_value = max_steering;
                    }
                    else if (steer_value < -max_steering)
                    {
                        steer_value = -max_steering;
                    }


                    throttle = target_throttle + pid_throttle.TotalError();



                    //we don't calculate error in first n itteration.
                    if(td.it < td.skip)
                    {
                        send_position(ws, steer_value, throttle);
                        return;
                    }

                    td.error += pow(cte, 2);
                    //punish throttle higher than 1
                    //and punish close to 0 throttle

                    if( td.first_run && (td.it >= td.n))
                    {
                        td.first_run = false;
                        td.best_error = td.error;
                        td.p[0] += td.dp[0];
                        td.it=0;
                        reset(ws);
                        return;
                    }
                    //if error is bigger then we can just stop
                    if (!td.first_run &&( td.it >= td.n || td.error> td.best_error || td.error/td.it > td.error_per_it ))
                    {
                        if( td.error< td.best_error &&  td.error/td.it < td.error_per_it )
                        {
                            td.best_error = td.error;
                            td.dp[td.dp_idx] *= 1.1;
                            td.dp_idx= (td.dp_idx+1)%td.dp.size();
                            td.second_time = false;

                            for(int i =0; i< td.p.size(); i++)
                            {
                                td.best_p[i]=td.p[i];
                            }

                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                        }
                        else if(!td.second_time)
                        {
                            td.p[td.dp_idx] -= 2 * td.dp[td.dp_idx];
                            td.second_time = true;
                        }
                        else
                        {
                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                            td.dp[td.dp_idx] *= 0.9;
                            td.second_time = false;
                            td.dp_idx= (td.dp_idx+1)%td.dp.size();
                            td.p[td.dp_idx] += td.dp[td.dp_idx];
                        }

                        td.it=0;
                        reset(ws);
                        return;
                    }
                    send_position(ws, steer_value, throttle);
                    // DEBUG




                }  // end "telemetry" if
            }
            else
            {
                // Manual driving
                manual(ws);
            }
        }  // end websocket message if
    }); // end h.onMessage
}
struct param
{
    bool faster = false;
};

param p;

void simulation(uWS::Hub & h, PID & pid, PID & pid_throttle, PID & pid_throttle_2, bool & faster_mode)
{

    //0.18502, 0.000324643, 5.18224
    pid.Init(0.18502, 0.000324643, 4.57675, false);

    //0.609181, 0, 0.685517
    //0.6 -1.75075e-05 0.3813
    //1.64954 -0.9 1.70107
    pid_throttle.Init(0.609181, 0, 0.685517, false);
    pid_throttle_2.Init(0.6, 0, 0.3813,false);

    p.faster = faster_mode;

    h.onMessage([&pid, &pid_throttle, &pid_throttle_2, &p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode)
    {
        double target_throttle = 0.7;
        double max_steering = 1;
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(string(data).substr(0, length));

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value;
                    double throttle;
                    /**
                     * TODO: Calculate steering value here, remember the steering value is
                     *   [-1, 1].
                     * NOTE: Feel free to play around with the throttle and speed.
                     *   Maybe use another PID controller to control the speed!
                     */
                    pid.UpdateError(cte);
                    if(!p.faster)
                    {
                        throttle = 0.3;
                    }
                    else
                    {
                        pid_throttle_2.UpdateError(cte);
                        pid_throttle.UpdateError(cte);

                        throttle = target_throttle + (pid_throttle.TotalError() + pid_throttle_2.TotalError())/2 ;
                    }

                    //throttle = 0.3;

                    steer_value = pid.TotalError();
                    if( steer_value > max_steering )
                    {
                        steer_value = max_steering;
                    }
                    else if (steer_value < -max_steering)
                    {
                        steer_value = -max_steering;
                    }
                    if(speed > 50 && fabs(steer_value)>0.5 && throttle>0)
                    {
                        throttle =-0.5;
                    }
                    // DEBUG

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    std::cout << msg << std::endl;
                    std::cout << "CTE: " << cte << " Steering Value: " << steer_value<< " Throttle Value: "<< throttle
                              << std::endl;

                }  // end "telemetry" if
            }
            else
            {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket message if
    }); // end h.onMessage

}

int main()
{
    uWS::Hub h;
    PID pid;
    PID pid_throttle;
    PID pid_throttle_2;
    twiddle_data td;
    /**
     * TODO: Initialize the pid variable.
     */
    bool train_mode = false;
    bool faster_mode = false;
    bool train_mode_throttle= true;
    bool train_mode_steering_with_throttle = false;
    //we need to pass h, pid, td
    //if we create twiddle_data inside twiddle it won't work.
    //
    if(train_mode)
    {
        if(train_mode_throttle)
        {
            twiddle_throttle(h,pid,pid_throttle,td);
        }
        else if(train_mode_steering_with_throttle)
        {
            //optimize steering with dynamic throttle;
            twiddle_steering(h,pid,pid_throttle,td);
        }
        else
        {
            twiddle(h, pid, td);
        }
    }
    else
    {
        simulation(h, pid, pid_throttle,pid_throttle_2, faster_mode);
    }


    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
