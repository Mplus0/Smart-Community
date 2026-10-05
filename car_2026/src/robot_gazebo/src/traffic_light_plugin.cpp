#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <functional>
#include <string>

namespace gazebo
{

class TrafficLightPlugin : public ModelPlugin
{
public:
    TrafficLightPlugin()
        : state_(RED),
          red_duration_(10.0),
          yellow_duration_(5.0),
          green_duration_(15.0)
    {
    }

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        model_ = model;
        world_ = model_->GetWorld();

        // 读取 SDF 中的时间参数
        if (sdf->HasElement("red_duration"))
            red_duration_ = sdf->Get<double>("red_duration");

        if (sdf->HasElement("yellow_duration"))
            yellow_duration_ = sdf->Get<double>("yellow_duration");

        if (sdf->HasElement("green_duration"))
            green_duration_ = sdf->Get<double>("green_duration");

        // Gazebo Transport
        node_.reset(new transport::Node());
        node_->Init(world_->Name());

        visual_pub_ =
            node_->Advertise<msgs::Visual>("~/visual");

        // 初始状态：红灯
        state_ = RED;
        last_switch_time_ = world_->SimTime();

        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                std::bind(
                    &TrafficLightPlugin::OnUpdate,
                    this));

        gzmsg << "[TrafficLightPlugin] Loaded for model: "
              << model_->GetName() << "\n";

        gzmsg << "[TrafficLightPlugin] red="
              << red_duration_
              << "s, yellow="
              << yellow_duration_
              << "s, green="
              << green_duration_
              << "s\n";
    }

private:

    enum LightState
    {
        RED,
        YELLOW,
        GREEN
    };

    void OnUpdate()
    {
        common::Time now = world_->SimTime();

        double elapsed =
            (now - last_switch_time_).Double();

        switch (state_)
        {
        case RED:
            if (elapsed >= red_duration_)
            {
                state_ = YELLOW;
                last_switch_time_ = now;

                ApplyState();

                gzmsg << "[TrafficLightPlugin] RED -> YELLOW\n";
            }
            break;

        case YELLOW:
            if (elapsed >= yellow_duration_)
            {
                state_ = GREEN;
                last_switch_time_ = now;

                ApplyState();

                gzmsg << "[TrafficLightPlugin] YELLOW -> GREEN\n";
            }
            break;

        case GREEN:
            if (elapsed >= green_duration_)
            {
                state_ = RED;
                last_switch_time_ = now;

                ApplyState();

                gzmsg << "[TrafficLightPlugin] GREEN -> RED\n";
            }
            break;
        }
    }

    void ApplyState()
    {
        switch (state_)
        {
        case RED:
            SetMaterial(
                "red_lamp",
                "TrafficLight/RedOn");

            SetMaterial(
                "yellow_lamp",
                "TrafficLight/YellowOff");

            SetMaterial(
                "green_lamp",
                "TrafficLight/GreenOff");
            break;

        case YELLOW:
            SetMaterial(
                "red_lamp",
                "TrafficLight/RedOff");

            SetMaterial(
                "yellow_lamp",
                "TrafficLight/YellowOn");

            SetMaterial(
                "green_lamp",
                "TrafficLight/GreenOff");
            break;

        case GREEN:
            SetMaterial(
                "red_lamp",
                "TrafficLight/RedOff");

            SetMaterial(
                "yellow_lamp",
                "TrafficLight/YellowOff");

            SetMaterial(
                "green_lamp",
                "TrafficLight/GreenOn");
            break;
        }
    }

    void SetMaterial(
        const std::string &visual_name,
        const std::string &material_name)
    {
        msgs::Visual msg;

        // 例如：
        // traffic_light::housing::red_lamp
        std::string parent =
            model_->GetName() + "::housing";

        std::string visual =
            parent + "::" + visual_name;

        msg.set_name(visual);
        msg.set_parent_name(parent);

        msgs::Material *material =
            msg.mutable_material();

        msgs::Material::Script *script =
            material->mutable_script();

        script->set_name(material_name);

        script->add_uri(
            "model://traffic_light/materials/scripts");

        script->add_uri(
            "model://traffic_light/materials/textures");

        visual_pub_->Publish(msg);
    }

private:
    physics::ModelPtr model_;
    physics::WorldPtr world_;

    transport::NodePtr node_;
    transport::PublisherPtr visual_pub_;

    event::ConnectionPtr update_connection_;

    LightState state_;

    common::Time last_switch_time_;

    double red_duration_;
    double yellow_duration_;
    double green_duration_;
};

GZ_REGISTER_MODEL_PLUGIN(TrafficLightPlugin)

}