#ifndef YOLOCLIENT_YOLOCLIENT_CPP
#define YOLOCLIENT_YOLOCLIENT_CPP

#include <pylike/argparse.h>
#include <dds/message/yolo/Objects.h>
#include <dds/pubsub/DDSPublisher.h>
#include <dds/conversion/opencv.h>

#include <opencv2/opencv.hpp>

#include "videoEncDec/opencv/capture.h"
#include "privateVideo/privateVideoTransfer.h"


#define nh lightdds::nodeHandle

argparse::ArgumentParser getArgs(int argc, char** argv)
{
    argparse::ArgumentParser parser("yoloClient_yoloClient parser", argc, argv);
    parser.add_argument({"-n", "--node-name"}, "yoloClient_node", "this node name");
    parser.add_argument({"--topic-name"}, "/camera/result", "topic name");
    parser.add_argument({"-i", "--ip"}, "127.0.0.1", "server ip");
    parser.add_argument({"-p", "--port"}, 21000, "server port");
    parser.add_argument({"--buffer-size"}, 5, "publisher max buffer size");
    parser.add_argument({"--debug"}, STORE_TRUE, "use debug mode");  // bool
    parser.add_argument({"--source"}, "/path/to/your/video", "video file path");
    parser.add_argument({"--drop"}, STORE_TRUE, "drop frame");
    parser.add_argument({"-t", "--type"}, "normal", "video type, normal/jpeg/yuyv/stream");
    parser.add_argument({"--send-image"}, STORE_TRUE, "send image");
    logaddAndSetFromParser2(parser);
    logsetStdoutFormat(((bool)parser["debug"])?"$TIME | $LEVEL | $LOCATION - $MSG":"$TIME | $MSG");
    return parser;
}


static inline cv::Mat static_resize(const cv::Mat& img, cv::Size input_size, float& r) {
    if (img.size().width == input_size.width && img.size().height == input_size.height)
    {
        r = 1.0;
        return img;
    }
    r = std::min(input_size.width / (img.cols*1.0), input_size.height / (img.rows*1.0));
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;

    cv::Mat re(unpad_h, unpad_w, CV_8UC3);
    cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
    cv::Mat out(input_size.height, input_size.width, CV_8UC3, cv::Scalar(114, 114, 114));
    re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
    return out;
}

class YOLOClient
{
public:
    YOLOClient() {}

    YOLOClient(std::string ip, int port): ip_(ip), port_(port)
    {
        client_.init(ip, port, true);
        
    }

    int connect(int delay=1, int times=0)
    {
        while (0 != client_.connectServer())
        {
            WARN << "connect server failed, try again" << ENDL;
            client_.init(ip_, port_);
            if (delay) sleep(delay);
            if (--times == 0) return -1;
        }

        // INFO << 1 << ENDL;
        client_.recvData(&input_size.width, sizeof(int));
        // INFO << 2 << ENDL;
        client_.recvData(&input_size.height, sizeof(int));
        // INFO << 3 << ENDL;
        
        names_.clear();
        int length_names=0;
        // INFO << 4 << ENDL;
        client_.recvData(&length_names, sizeof(int));
        char* names = new char[length_names];
        // INFO << 5 << ENDL;
        client_.recvData(names, length_names);
        // INFO << 6 << ENDL;
        pystring names_str(names);
        names_ = names_str.split(",");
        // for (auto name: names_)
        // {
        //     printf("%s\n", name.c_str());
        // }
        return 0;
    }

    bool isConnected()
    {
        return client_.isConnect();
    }

    void pushImg(cv::Mat& img)
    {
        float ratio = 1.0;
        cv::Mat frame_resize = static_resize(img, input_size, ratio);
        imgs.push(img);
        ratios.push(ratio);
        client_.sendData(frame_resize.data, frame_resize.total() * frame_resize.elemSize());
    }


    std::vector<std::vector<float>> getResult(cv::Mat& img, bool draw_box=false)
    {
        int numResults = 0;
        std::vector<std::vector<float>> result;
        std::vector<float> result2recv;
        int ret = client_.recvData(&numResults, sizeof(int));
        if (ret != 0) return result;
        
        img = imgs.front();
        // std::cout << img.size() << std::endl;
        imgs.pop();
        float ratio = ratios.front();
        ratios.pop();

        
        if (numResults)
        {
            result.resize(numResults);
            result2recv.resize(numResults * 6);
            INFO << 5 << ENDL;
            client_.recvData(result2recv.data(), 6 * sizeof(float) * numResults);
            INFO << 6 << ENDL;
            
            for (int i=0;i<numResults;i++)
            {
                result[i].resize(6);
                for (int j=0;j<6;j++)
                {
                    result[i][j] = result2recv[i * 6 + j];
                    if (j < 4)
                    {
                        result[i][j] /= ratio;
                    }
                }
            }
            if (draw_box)
            {
                draw(img, result);
            }
        }
        
        return result;
    }

    std::vector<std::vector<float>> detect(cv::Mat& frame, bool draw_box=false)
    {
        float ratio = 1.0; // std::min((float)input_size.width / frame.cols, (float)input_size.height / frame.rows);

        cv::Mat frame_resize = static_resize(frame, input_size, ratio);

        // INFO << ratio << ENDL;

        // cv::resize(frame, frame_resize, input_size);
        client_.sendData(frame_resize.data, frame_resize.total() * frame_resize.elemSize());

        int numResults = 0;
        client_.recvData(&numResults, sizeof(int));
        std::vector<std::vector<float>> result;
        std::vector<float> result2recv;
        
        if (numResults)
        {
            result.resize(numResults);
            result2recv.resize(numResults * 6);
            client_.recvData(result2recv.data(), 6 * sizeof(float) * numResults);
            
            for (int i=0;i<numResults;i++)
            {
                result[i].resize(6);
                for (int j=0;j<6;j++)
                {
                    result[i][j] = result2recv[i * 6 + j];
                    if (j < 4)
                    {
                        result[i][j] /= ratio;
                    }
                }
            }
            if (draw_box)
            {
                draw(frame, result);
            }
        }
        
        return result;
    }

    void close()
    {
        client_.closeClient();
    }

private:
    cv::Size input_size;
    std::vector<pystring> names_;
    std::string ip_;
    privateVideo::Client client_;
    std::queue<cv::Mat> imgs;
    std::queue<float> ratios;
    int port_ = 0;
    void* cap_=nullptr;

    static cv::Scalar get_color(int index){
        static const int color_list[][3] = {
            {255, 56, 56},{255, 157, 151},{255, 112, 31},{255, 178, 29},{207, 210, 49},
            {72, 249, 10},{146, 204, 23},{61, 219, 134},{26, 147, 52},{0, 212, 187},
            {44, 153, 168},{0, 194, 255},{52, 69, 147},{100, 115, 255},{0, 24, 236},
            {132, 56, 255},{82, 0, 133},{203, 56, 255},{255, 149, 200},{255, 55, 198}
        };
        index %= 20;
        return cv::Scalar(color_list[index][2], color_list[index][1], color_list[index][0]);
    }


    void draw(cv::Mat& dist, std::vector<std::vector<float>> prediction, bool draw_label=true, int thickness=20)
    {
        cv::Scalar color;
        cv::Scalar txt_color;
        cv::Scalar txt_bk_color;

        cv::Size label_size;

        int baseLine = 0;
        int x1, y1, x2, y2, out_point_y;
        int line_thickness = std::round((double)thickness / 10.0);
        
        for(int k=0; k<prediction.size(); k++){
            int label_ = (int)prediction.at(k)[4];
            color = get_color(label_);

            x1 = prediction.at(k)[0];
            y1 = prediction.at(k)[1];
            x2 = prediction.at(k)[2];
            y2 = prediction.at(k)[3];

            cv::rectangle(
                dist,
                cv::Rect2f(x1, y1, x2-x1, y2-y1),
                color,
                line_thickness
            );
            
            if (draw_label){
                txt_color = (cv::mean(color)[0] > 127)?cv::Scalar(0, 0, 0):cv::Scalar(255, 255, 255);
                std::string label = names_.at(label_) + " " + std::to_string(prediction.at(k)[5]).substr(0, 4);
                label_size = cv::getTextSize(label.c_str(), cv::LINE_AA, double(thickness) / 30.0, (line_thickness>1)?line_thickness-1:1, &baseLine);
                txt_bk_color = color; // * 0.7;
                y1 = (y1 > dist.rows)?dist.rows:y1 + 1;
                out_point_y = y1 - label_size.height - baseLine;
                if (out_point_y >= 0) y1 = out_point_y;
                cv::rectangle(dist, cv::Rect(cv::Point(x1 - (line_thickness - 1), y1), cv::Size(label_size.width, label_size.height + baseLine)),
                            txt_bk_color, -1);
                cv::putText(dist, label, cv::Point(x1, y1 + label_size.height),
                            cv::LINE_AA, double(thickness) / 30.0, txt_color, (line_thickness>1)?line_thickness-1:1);
            }

        }
    }
};

void pushThread(BaseCapture* cap, YOLOClient* client, bool* stop_)
{
    while (cap->isOpened() && !*stop_)
    {
        cv::Mat frame;
        *cap >> frame;
        if (frame.empty()) 
        {
            *stop_ = true;
            break;
        }
        INFO << "push" << ENDL;
        client->pushImg(frame);
    }
}


int main(int argc, char** argv)
{
    // init params
    auto args = getArgs(argc, argv);
    std::string node_name = args["node-name"];
    std::string topic_name = args["topic-name"];
    int max_buffer_size = args["buffer-size"];
    bool debug = args["debug"];
    std::string source = args["source"];
    std::string sourceType = args["type"];
    bool drop = args["drop"];
    bool send_image = args["send-image"];

    YOLOClient client(args["ip"], args["port"]);
    client.connect();
    
    int type = 0;
    if (sourceType == "normal")
    {
        type = Capture::CAP_TYPE_NORMAL;
    }
    else if (sourceType == "jpeg")
    {
        type = Capture::CAP_TYPE_JPEG;
    }
    else if (sourceType == "yuyv")
    {
        type = Capture::CAP_TYPE_YUYV;
    }
    else if (sourceType == "stream")
    {
        type = Capture::CAP_TYPE_STREAM;
    }
    else
    {
        ERROR << "unknown source type: " << sourceType << ENDL;
        return -1;
    }
    BaseCapture* cap = Capture::createCapture(source, cv::CAP_ANY, type, drop);
    // cv::VideoCapture cap(source);
    if (sourceType == "jpeg" || sourceType == "normal")
    {
        cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap->set(cv::CAP_PROP_FPS, 30);
        cap->set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap->set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }
    

    lightdds::DDSPublisher* pub = nh::advertise<yolo::Objects>(topic_name, max_buffer_size, "yoloClient_pub_node1");

    if (!cap->isOpened())
    {
        ERROR << "can not open " << source << ENDL;
        return -1;
    }
    INFO << "fps:" << cap->get(cv::CAP_PROP_FPS) << ENDL;
    cv::Mat frame;
    bool stop = false;
    
    INFO << 1 << ENDL;

    std::thread pushT(&pushThread, cap, &client, &stop);

    INFO << 2 << ENDL;
    yolo::Object obj;
    yolo::Objects msg;
    msg.header.frame_id = topic_name;

    msg.image_width = cap->get(cv::CAP_PROP_FRAME_WIDTH);
    msg.image_height = cap->get(cv::CAP_PROP_FRAME_HEIGHT);
    msg.send_image = send_image;
    dds_conversion::EncodeParams params;
    params.encoding_type = dds_conversion::ENCODE_TYPE_JPEG;
    params.image_quality = 90;

    

    uint64_t count = 0;
    while (cap->isOpened() && client.isConnected() && !stop)
    {
        auto t0 = pytime::time();

        
        INFO << "get result" << ENDL;
        auto result = client.getResult(frame, true);
        INFO << "get result done" << ENDL;

        msg.objects.clear();
        
        for (auto r: result)
        {
            obj.x = r[0];
            obj.y = r[1];
            obj.width = r[2] - r[0];
            obj.height = r[3] - r[1];
            obj.label = r[4];
            obj.confidence = r[5];
            msg.objects.push_back(obj);
        }
        if (send_image)
        {
            dds_conversion::fromOpenCVMat(frame, msg.image, params);
        }
        msg.image_id = count++;
        pub->publish(msg);


        double dt = pytime::time() - t0;
        INFO << "detect time:" << dt << "s" << ENDL;

    }
    cv::destroyAllWindows();
    stop = true;
    pushT.join();
    cap->release();
    client.close();
    
    return 0;
}


#endif /* YOLOCLIENT_YOLOCLIENT_CPP */

