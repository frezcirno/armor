// by zhongzebin 2020/07/13

//先找核心函数 run 然后再看其他流程

#ifndef WINDMILL_HPP
#define WINDMILL_HPP


#include "base.hpp"
#include "imageshow.hpp"
#include "Target.hpp"
#include "Tool.hpp"
#include "opencv2/opencv.hpp"
#include <algorithm>

#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/framework/tensor.h"
#include "google/protobuf/wrappers.pb.h"
#include "tensorflow/core/util/command_line_flags.h"

using tensorflow::string;
using tensorflow::Tensor;

using namespace tensorflow;

#include <stdlib.h>

namespace wm {
    const cv::String PATH = "../data/windmill/";
    const cv::String FILE_TYPE = ".jpg";
    static int gNameCount = 0;
    typedef enum {
        TRACK, JUMP
    } State;

    class Windmill {
    public:
        bool run(const cv::Mat &frame, float &pitch, float &yaw,
                 double time);

        static Windmill *GetInstance(const cv::Mat &cam, const cv::Mat &dist,
                                     const cv::Mat &TvCtoL, const double delay, const cv::String &modelName,
                                     armor::ImageShowClient *is, const double maxPitchError, const double maxYawError) {
            static Windmill instance(cam, dist, TvCtoL, delay, modelName, is, maxPitchError, maxYawError);
            return &instance;
        }
        std::string mode;
        State state = JUMP;
        Target findedTarget;
        Target hitTarget;
        cv::Point3f center;
        cv::Point2f center2D;
        double timeStamp = 0.0;
        double delay;
        cv::Mat camMatrix;
        cv::Mat distCoeffs;
        float max_differ=0.08;
        float min_differ=0.04;
        int loop;
        std::vector<float> angle_list;
        std::vector<double> timeStamp_list;
        cv::Mat TvCtoL;

        double maxPitchError = 0.0;
        double maxYawError = 0.0;

        bool close;
        bool isSmallROI = false;

        tensorflow::Session* session;
        int fixedSize=32;
        tensorflow::Tensor input;
        const string input_name = "input_1:0";
        const string output_name = "y/Sigmoid:0";

        int pos_count=0,neg_count=0;

    private:
        cv::Mat threshold(cv::Mat frame);
        
        void pre_process(cv::Mat& gray);
        bool find_items(cv::Mat gray,cv::RotatedRect& Rrect,cv::Point& windmill_c,cv::Mat thre);
        void find_armor(cv::RotatedRect Rrect,cv::Point& armor,float& angle,float& length,cv::Point windmill_c);
        bool find_center(cv::Point& windmill_c,std::vector<std::vector<cv::Point>> contours,int i,cv::Mat thre);
        explicit Windmill(const cv::Mat &cam, const cv::Mat &dist, const cv::Mat &TvCtoL,
                          const double delay, const cv::String &modelName, armor::ImageShowClient *is,
                          const double maxPitchError, const double maxYawError)
                : delay(delay), camMatrix(cam), distCoeffs(dist), TvCtoL(TvCtoL), is(is), maxPitchError(maxPitchError),
                  maxYawError(maxYawError) {
            //--------------------------------创建session------------------------------
            tensorflow::Status status = tensorflow::NewSession(tensorflow::SessionOptions(), &session);
            if (!status.ok()) {
                    std::cout << status.ToString() << std::endl;
            } else {
                    std::cout << "Session created successfully" << std::endl;
            }
            const string model_path = "../happyModel.pb";
            //--------------------------------从pb文件中读取模型--------------------------------

            tensorflow::GraphDef graph_def;
            //读取Graph, 如果是文本形式的pb,使用ReadTextProto
            status = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), model_path, &graph_def);
            if (!status.ok()) {
                    std::cout << status.ToString() << std::endl;
            } else {
                    std::cout << "Load graph protobuf successfully" << std::endl;
            }
            //--------------------------------将模型设置到创建的Session里--------------------------------
            status = session->Create(graph_def);
            if (!status.ok()) {
                    std::cout << status.ToString() << std::endl;
            } else {
                    std::cout << "Add graph to session successfully" << std::endl;
            }

            input=tensorflow::Tensor (tensorflow::DT_FLOAT, tensorflow::TensorShape({ 1, fixedSize, fixedSize, 1 }));
        }
        void generate_list(float angle);
        void pnp(cv::Point armor,double length,float angle,float &pitch, float &yaw);
        bool find_light(std::vector<std::vector<cv::Point>> contours,cv::RotatedRect& right,std::vector<cv::Vec4i> hierarchy,cv::Point windmill_c);
        void mat2Tensor(cv::Mat &image, Tensor &t);

        armor::ImageShowClient *is;
    };

    //问题：必须拿到风车匀速转动代码进行调试！（新风车已经给出了转速函数，因此不需要再过多到依赖于历史列表，只需要确保获得准确风车已经运行时间到时间以及弹丸飞行到时间即可准确预测）
    void Windmill::generate_list(float angle)//generate angle list and time stamp list
		{
			if(angle_list.empty())
			{
				angle_list.push_back(angle);
		          timeStamp_list.push_back(timeStamp);
				std::cout<<"angle:"<<angle<<endl;
				return;
			}
			else
			{
				is->addText(cv::format("angle num: %d",angle_list.size()));
				if(maxYawError>0)//clockwise
				{
					if(angle*angle_list[angle_list.size()-1]>0)//同号情况,angle增
					{
						if(angle-angle_list[angle_list.size()-1]<max_differ && angle-angle_list[angle_list.size()-1]>-min_differ)
			              {
			                  angle_list.push_back(angle);
			                  timeStamp_list.push_back(timeStamp);
		                      std::cout<<"angle:"<<angle<<endl;
			                  return;
			              }
					}
					else//异号情况
          {
						if(angle_list[angle_list.size()-1]>3.14-max_differ) //上一帧比3.14-differ大
						{
                if (6.28+angle - angle_list[angle_list.size() - 1] < max_differ && 6.28+angle - angle_list[angle_list.size() - 1] > -min_differ)//当前帧应当+6.28
                {
                  	angle_list.push_back(angle);
                    timeStamp_list.push_back(timeStamp);
                    std::cout<<"angle:"<<angle<<endl;
                  	return;
                }
            }
            else if(angle-angle_list[angle_list.size()-1]<max_differ && angle-angle_list[angle_list.size()-1]>-min_differ)//上一帧比0小，依旧递增原则
            {
          			angle_list.push_back(angle);
                timeStamp_list.push_back(timeStamp);
                std::cout<<"angle:"<<angle<<endl;
              	return;
            }
          }
        }
				else//anti-clockwise
		          {
		          	//is->addText(cv::format());
		              if(angle*angle_list[angle_list.size()-1]>0)//同号情况,angle减
		              {
		                  if(angle-angle_list[angle_list.size()-1]>-max_differ && angle-angle_list[angle_list.size()-1]<min_differ)
		                  {
		                      angle_list.push_back(angle);
		                      timeStamp_list.push_back(timeStamp);
		                      std::cout<<"angle:"<<angle<<endl;
		                      return;
		                  }
		              }
		              else//异号情况
		              {
		                  if(angle_list[angle_list.size()-1]<-3.14+max_differ) //上一帧比-3.14+differ小
		                  {
		                      if (-6.28+angle - angle_list[angle_list.size() - 1] > -max_differ && -6.28+angle - angle_list[angle_list.size() - 1] < min_differ)//当前帧应当-6.28
		                      {
		                          angle_list.push_back(angle);
		                          timeStamp_list.push_back(timeStamp);
		                          std::cout<<"angle:"<<angle<<endl;
		                          return;
		                      }
		                  }
		                  else if(angle-angle_list[angle_list.size()-1]>-max_differ && angle-angle_list[angle_list.size()-1]<min_differ)//上一帧比0小，依旧递增原则
		                  {
		                      angle_list.push_back(angle);
		                      timeStamp_list.push_back(timeStamp);
		                      std::cout<<"angle:"<<angle<<endl;
		                      return;
		                  }
		              }
		          }
			}
			//如果不满足上述条件，则视作击打扇叶发生了改变，清零重新检测
			angle_list.clear();
            timeStamp_list.clear();
			std::cout<<"not valid angle"<<endl;
			loop=0;
			return;
		}
    void Windmill::pnp(cv::Point armor,double length,float angle,float &pitch, float &yaw)//calculate pitch and yaw
    {
        std::vector<cv::Point2f> armor_p;
        cv::Point p1;
        p1.x=armor.x-length*0.2*sin(angle);
        p1.y=armor.y+length*0.2*cos(angle);
        p1.x=p1.x+length*0.16*cos(angle);
        p1.y=p1.y+length*0.16*sin(angle);
        armor_p.push_back(p1);
        p1.x=p1.x+length*0.4*sin(angle);
        p1.y=p1.y-length*0.4*cos(angle);
        armor_p.push_back(p1);
        p1.x=p1.x-length*0.32*cos(angle);
        p1.y=p1.y-length*0.32*sin(angle);
        armor_p.push_back(p1);
        p1.x=p1.x-length*0.4*sin(angle);
        p1.y=p1.y+length*0.4*cos(angle);
        armor_p.push_back(p1);
        is->addText(cv::format("angle: %f",angle));
        cv::RotatedRect armor_rect(armor,cv::Size(0.32*length,0.4*length),angle*360/CV_2PI);
        std::vector<cv::RotatedRect> Rrect_list;
        Rrect_list.push_back(armor_rect);
        is->addRotatedRects("find armor",Rrect_list);
        is->addCircle("find armor center",armor);
        std::vector<cv::Point3f> armor_t;
        armor_t.push_back(cv::Point3f(-150, -125, 0));
        armor_t.push_back(cv::Point3f(150, -125, 0));
        armor_t.push_back(cv::Point3f(150, 125, 0));
        armor_t.push_back(cv::Point3f(-150, 125, 0));
        cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::solvePnP(armor_t, armor_p,camMatrix,distCoeffs,rVec,tVec);
        float x=tVec.ptr<double>(0)[0]+TvCtoL.ptr<double>(0)[0];
        float y=tVec.ptr<double>(1)[0]+TvCtoL.ptr<double>(1)[0];
        float z=tVec.ptr<double>(2)[0]+TvCtoL.ptr<double>(2)[0];
        pitch=acos(sqrt(pow(x,2)+pow(z,2))/sqrt(pow(x,2)+pow(y,2)+pow(z,2)));
        if(y<0) pitch=-pitch;
        yaw=acos(sqrt(pow(y,2)+pow(z,2))/sqrt(pow(x,2)+pow(y,2)+pow(z,2)));
        if(x<0) yaw=-yaw;
        is->addText(cv::format("pitch: %f",pitch));
        is->addText(cv::format("yaw: %f",yaw));
        is->addText(cv::format("x: %d",int(x)));
        //std::cout<<"x:"<<int(tVec.ptr<double>(0)[0])<<endl;
        is->addText(cv::format("y: %d",int(y)));
        //std::cout<<"y:"<<int(tVec.ptr<double>(1)[0])<<endl;
        is->addText(cv::format("z: %d",int(z)));
        //std::cout<<"z:"<<int(tVec.ptr<double>(2)[0])<<endl;
    }
    void Windmill::find_armor(cv::RotatedRect Rrect,cv::Point& armor,float& angle,float& length,cv::Point windmill_c)//find the target armor after prediction 利用极坐标，给定预测角度来获得预测后到装甲板中心坐标
    {
        int dis_x=Rrect.center.x-windmill_c.x;
        int dis_y=Rrect.center.y-windmill_c.y;

        angle=atan(float(dis_y)/float(dis_x));
        length=sqrt(dis_x*dis_x+dis_y*dis_y);
        if(dis_x<0)
        {
            if(dis_y>0)
                angle=CV_PI+angle;
            else
                angle=-CV_PI+angle;
        }
        if(mode=="linear")
            angle=angle+delay*CV_PI/30;
        if(mode=="triangular")
        {
            generate_list(angle);
            int pre_frame=10;
            if(angle_list.size()>=pre_frame)
            {
                float v;
                if(maxYawError>0) {
                    if(angle_list[angle_list.size() - 1] > angle_list[angle_list.size() - pre_frame])
                        v = (angle_list[angle_list.size() - 1] - angle_list[angle_list.size() - pre_frame]) /
                            (timeStamp_list[angle_list.size() - 1] - timeStamp_list[angle_list.size() - pre_frame])*1000;
                    else v=(CV_2PI+angle_list[angle_list.size() - 1] - angle_list[angle_list.size() - pre_frame]) /
                           (timeStamp_list[angle_list.size() - 1] - timeStamp_list[angle_list.size() - pre_frame])*1000;
                }
                else{
                    if(angle_list[angle_list.size() - 1] < angle_list[angle_list.size() - pre_frame])
                        v = (angle_list[angle_list.size() - 1] - angle_list[angle_list.size() - pre_frame]) /
                            (timeStamp_list[angle_list.size() - 1] - timeStamp_list[angle_list.size() - pre_frame])*1000;
                    else v=(-CV_2PI+angle_list[angle_list.size() - 1] - angle_list[angle_list.size() - pre_frame]) /
                           (timeStamp_list[angle_list.size() - 1] - timeStamp_list[angle_list.size() - pre_frame])*1000;
                }
                std::cout<<"v:"<<v<<endl;
            }
        }
        if(angle>CV_PI) angle-=CV_2PI;
        if(angle<-CV_PI) angle+=CV_2PI;
        armor.x=int(length*1.45*cos(angle)+windmill_c.x);
        is->addText(cv::format("length: %f",length));
        armor.y=int(length*1.45*sin(angle)+windmill_c.y);
        is->addText(cv::format("armor.y: %d",armor.y));
    }
    bool Windmill::find_light(std::vector<std::vector<cv::Point>> contours,cv::RotatedRect& right,std::vector<cv::Vec4i> hierarchy,cv::Point windmill_c)//find target flow bar 不可用，新规灯条形状未变，后续再改
    {
        int match=-1;
        int distance=999999999;
        for (int i = 0; i >= 0; i = hierarchy[i][0])
        {
            if (contours[i].size() < 5)
                continue;
            right = cv::fitEllipse(contours[i]);
            if (right.size.area() < 1800)
                continue;
            if((float(right.size.height) / right.size.width > 1.4 || float(right.size.width) / right.size.height > 1.4))
            {
                int dis_x=right.center.x-windmill_c.x;
                int dis_y=right.center.y-windmill_c.y;
                int current_dis=sqrt(dis_x*dis_x+dis_y*dis_y);
                if(current_dis<distance)
                {
                    distance=current_dis;
                    match=i;
                }
            }
        }
        if(match!=-1)
        {
            right = cv::fitEllipse(contours[match]);
            return true;
        }
        return false;
    }

    cv::Mat Windmill::threshold(cv::Mat frame)//threshold颜色二值化，红色两段范围两个mask相加提高精度
    {
        cv::Mat hsv,gray,mask0,mask1;
        std::vector<cv::Mat> hsvsplit;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, hsvsplit);
        if (armor::stConfig.get<std::string>("attack.attack-color") == "red")//deal with the target_color
        {
            int LowH = 156;
            int LowS = 43;
            int LowV = 80;
            int HighH = 180;
            int HighS = 255;
            int HighV = 255;
            cv::inRange(hsv, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), mask0);//threshold value between low and high
            LowH = 0;
            HighH = 10;
            cv::inRange(hsv, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), mask1);
            gray=mask1;
            gray = mask0 + mask1;
        }
        //cv::imwrite("../gray.jpg",gray);
        return gray;
    }
    void Windmill::pre_process(cv::Mat& gray)//dilate & morphologyEx 膨胀腐蚀是否使用待定
    {
        cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::dilate(gray, gray, element1);
        cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, element2);
    }
    void Windmill::mat2Tensor(cv::Mat &img, Tensor &t) {
    		if(img.cols<img.rows)
				    resize(img,img,{int(img.cols*1.0/img.rows*fixedSize),fixedSize});
				else
				    resize(img,img,{fixedSize,int(img.rows*1.0/img.cols*fixedSize)});
		    cv::Mat blank=cv::Mat(cv::Size(fixedSize,fixedSize), img.type(), cv::Scalar(0));
		    cv::Mat imageROI=blank(cv::Rect(0,0,img.cols,img.rows));
    		img.copyTo(imageROI, img);
    		float *tensor_data_ptr = t.flat<float>().data();
				cv::Mat fake_mat(blank.rows, blank.cols, CV_32FC(blank.channels()), tensor_data_ptr);
				blank.convertTo(fake_mat, CV_32FC(blank.channels()));
    }
    bool Windmill::find_center(cv::Point& windmill_c,std::vector<std::vector<cv::Point>> contours,int i,cv::Mat thre)//find windmill center 由于相机在云台上，风车圆心在移动，为了准确预测需要重复定位风车的中心
    {
        cv::Rect rect;
        if (contours[i].size() < 5)
            return false;
        rect=cv::boundingRect(contours[i]);
        //is->addRect("find rect",rect);
        if (float(rect.height) / rect.width < 1.4 && float(rect.width) / rect.height < 1.4 && rect.area() < 2800 && rect.area() > 500)//该写法泛化能力较差，是针对视频而言，需要进一步修改
        {
            double m=-1;
            float result_predict =-1;
            cv::Mat roi;
            cv::Mat result;
            roi = thre(rect);
            mat2Tensor(roi, input);
            std::vector<tensorflow::Tensor> outputs;
            TF_CHECK_OK(session->Run({std::pair<string, tensorflow::Tensor>(input_name, input)}, {output_name}, {}, &outputs));
            //获取输出
            auto output_c = outputs[0].scalar<float>();
            result_predict = output_c();
            if(result_predict>0.5)
            {
                windmill_c.x = rect.tl().x + rect.width / 2;
                windmill_c.y = rect.tl().y + rect.height / 2;
                cv::imwrite("../pos/"+std::to_string(pos_count)+".jpg",roi);
                pos_count++;
                return true;
            }
            cv::imwrite("../neg/"+std::to_string(neg_count)+".jpg",roi);
            neg_count++;
        }
        return false;
    }
    bool Windmill::find_items(cv::Mat gray,cv::RotatedRect& Rrect,cv::Point& windmill_c,cv::Mat thre)//find the target light and windmill center
    {
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point>> contours;
        //extract the outlines of the image and save the points into contours
        cv::findContours(gray, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
        if (hierarchy.size())
            for (int i = 0; i >= 0; i = hierarchy[i][0])
            {
                if (find_center(windmill_c, contours, i,thre)) {
                    //is->addImg("test", thre, true);
                    if (find_light(contours, Rrect, hierarchy, windmill_c))
                        return true;
                }
            }
        return false;
    }
    bool Windmill::run(const cv::Mat &frame, float &pitch, float &yaw,
                       double time) {
        //清空历史
        findedTarget.clear();
        hitTarget.clear();
        //获得时间
        timeStamp = time;
        cv::Mat gray;
        gray=threshold(frame);
        cv::Mat thre;
        gray.copyTo(thre);
        pre_process(gray);
        is->addImg("test", gray, true);
        cv::RotatedRect Rrect;
        cv::Point windmill_c;
        std::vector<cv::RotatedRect> Rrect_list;
        if(find_items(gray,Rrect,windmill_c,thre))
        {
            Rrect_list.push_back(Rrect);
            is->addRotatedRects("find fan",Rrect_list);
            is->addCircle("find windmill center",windmill_c);
            cv::Point armor;
            float angle,length;
            find_armor(Rrect,armor,angle,length,windmill_c);
            //is->addText(cv::format("angle: %f",angle));
            //angle_list.push_back(angle);
            //if(angle_list.size()>1) is->addText(cv::format("angle differ: %f",angle_list[angle_list.size()-1]-angle_list[angle_list.size()-2]));
            pnp(armor,length,angle,pitch,yaw);
            STATE(INFO, "find target", 1)
		    isSmallROI = false;
		    return true;
        }
        else
        {
            std::cout<<"angle doesn't find"<<endl;
            angle_list.clear();
            timeStamp_list.clear();
            loop=0;
        }
        STATE(INFO, "no target", 1)
        isSmallROI = false;
        return false;
    }

} // namespace wm
#endif
