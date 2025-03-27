#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    rclcpp::shutdown();
    exit(0);
}

class TeleopControl : public rclcpp::Node {
    public:
        TeleopControl() : Node("kb_bot_teleop_node"), linear_(0), angular_(0), l_scale_(2.0), a_scale_(2.0) {
            twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        }

        void keyLoop() {
            char c;
            bool keyPress = false;
            
            tcgetattr(kfd, &cooked);
            memcpy(&raw, &cooked, sizeof(struct termios));
            raw.c_lflag &=~ (ICANON | ECHO);

            raw.c_cc[VEOL] = 1;
            raw.c_cc[VEOF] = 2;
            tcsetattr(kfd, TCSANOW, &raw);

            for(;;) {
                if(::read(kfd, &c, 1) < 0) {
                    perror("read():");
                    exit(-1);
                }

                linear_ = 0;
                angular_ = 0;

                switch (c) {
                    case 'a': case 'A': angular_ = 1.5; keyPress = true; break;
                    case 'd': case 'D': angular_ = -1.5; keyPress = true; break;
                    case 'w': case 'W': linear_ = 1.5; keyPress = true; break;
                    case 's': case 'S': linear_ = -1.5; keyPress = true; break;
                    case 'x': case 'X': linear_ = angular_ = 0.0; keyPress = true; break;
                    case 'q': case 'Q':
                        linear_ = angular_ = 0.0;
                        tcsetattr(kfd, TCSANOW, &cooked);  // Restore terminal settings.
                        rclcpp::shutdown();  // Shutdown ROS node.
                        exit(0);
                }

                geometry_msgs::msg::Twist msg;
                msg.angular.z = a_scale_ * angular_;
                msg.linear.x = l_scale_ * linear_;

                if (keyPress) {
                    twist_pub_->publish(msg);
                    keyPress = false;
                }
            }

            return;
        }
    
    private:
        std::shared_ptr<rclcpp::Node> nh_;
        double linear_, angular_, l_scale_, a_scale_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // auto node = rclcpp::Node::make_shared("kb_bot_teleop_node");

    // TeleopControl kb_bot_teleop_node(node);

    TeleopControl kb_bot_teleop_node;
    signal(SIGINT, quit);

    kb_bot_teleop_node.keyLoop();
    return(0);
}
