//
// Created by truth on 2020/3/8.
//
//layers and their functions
#include "tools.hpp"

#ifndef MYCNN_LAYERS_HPP
#define MYCNN_LAYERS_HPP

#endif //MYCNN_LAYERS_HPP



//层次信息
class layers{
private:
    std::string layerType;
    int layerNum = 0;
    //std::vector<cv::Mat> layerInfo;
public:
    std::vector<std::vector<cv::Mat>> W;
    std::vector<mytype>b;
    layers(std::string type,int num)
            :layerType(std::move(type)),layerNum(num)
    {}
    //虚函数多态
    //载入权重信息
    virtual bool loadWeights(std::ifstream &fin)=0;
    //显示权重信息
    virtual void showWeights() = 0;
    //计算该层输出
    virtual std::vector<cv::Mat> compute(const std::vector<cv::Mat>& src,bool showInfo=0) = 0;
    virtual void showBias() = 0;
    hparameters hparas{};
};

class LayerConv2D:public layers{
private:

public:
    LayerConv2D(std::string type,int num)
            :layers(std::move(type),num){

    }
    bool loadWeights(std::ifstream &fin) override{
        char tmp_char = ' ';
        std::string tmp_str,m_border_mode;
        float tmp_float;
        bool skip = false;
        fin >> hparas.output >> hparas.input >> hparas.filterSize >> hparas.filterSize >> m_border_mode>>hparas.stride;
        if (m_border_mode == "[") {//万一读错
            m_border_mode = "valid";
            hparas.pad=0;
            skip = true;
        }
        else if(m_border_mode == "valid"){//same
            hparas.pad=0;
        }
        // 读 weights
        for(int k = 0; k < hparas.output; ++k) {
            std::vector<cv::Mat>tmp_single_depth;
            for(int d = 0; d < hparas.input; ++d) {
                cv::Mat tmp_single_layer=cv::Mat::zeros(hparas.filterSize,hparas.filterSize,CV_64F);
                for(int r = 0; r < hparas.filterSize; ++r) {
                    if (!skip) { fin >> tmp_char; } // for '['
                    else { skip = false;}
                    for(int c = 0; c < hparas.filterSize; ++c) {

                        fin >> tmp_float;
                        tmp_single_layer.at<mytype >(r,c)=tmp_float;
                    }
                    fin >> tmp_char; // for ']'
                }
                tmp_single_depth.push_back(tmp_single_layer.clone());
            }
            W.push_back(tmp_single_depth);
        }

        // 读 biases
        fin >> tmp_char; // for '['
        for(int k = 0; k < hparas.output; ++k) {
            fin >> tmp_float;

            b.push_back(tmp_float);
        }
        //std::cout<<b[0][0]<<std::endl;
        fin >> tmp_char; // for ']'

    }
    void showWeights() override {
        int filter=W.size();
        int depth=W[0].size();
        for(int i=0;i<filter;i++){
            for(int j=0;j<depth;j++){
                std::cout<<"第"<<i<<"个，第"<<j<<"层:"<<std::endl;
                std::cout<<W[i][j]<<std::endl;
            }
        }
    }
    std::vector<cv::Mat> compute(const std::vector<cv::Mat>& src,bool showInfo) override{


        std::vector<cv::Mat> temp=convForward(src,W,b,hparas);
        //std::cout<<temp.size()<<std::endl;
        if(showInfo==1)
            for(int i=0;i<temp.size();i++){
                cv::Mat tmp=temp[i];
                std::cout<<"this is LayerConv2D"<<std::endl;
                std::cout<<"第"<<i<<"个："<<tmp.size<<"\n"<<tmp<<std::endl;
            }

        return temp;
    }
    void showBias() override{
        for(int i=0;i<b.size();i++){
            std::cout<<"第"<<i<<"层:"<<std::endl;
            std::cout<<b[i]<<std::endl;
        }
    }
};

class LayerActivation:public layers{
private:
    std::string m_activation_type;
public:
    LayerActivation(std::string type,int num)
            :layers(std::move(type),num){
    }
    bool loadWeights(std::ifstream &fin) override{
        fin >> m_activation_type;
    }
    void showWeights() override {
        std::cout<<"hello"<<std::endl;
    }
    void showBias() override{;}
    std::vector<cv::Mat> compute(const std::vector<cv::Mat>& src,bool showInfo) override{


        std::vector<cv::Mat> temp;
        for(int i=0;i<src.size();i++){
            cv::Mat tmp;
            if(m_activation_type=="relu")
                tmp=relu(src[i]);
            if(m_activation_type=="sigmoid")
                tmp=sigmoid(src[i]);
            //std::cout<<i<<std::endl;
            temp.push_back(tmp);
            if(showInfo==1){
                std::cout<<"this is active"<<std::endl;
                std::cout<<src[i]<<std::endl;
                std::cout<<"第"<<i<<"个："<<tmp.size<<"\n"<<tmp<<std::endl;

            }
        }
        return temp;
    }
};

class MaxPooling2D:public layers{
private:
    std::string m_activation_type;
public:
    MaxPooling2D(std::string type,int num)
            :layers(std::move(type),num){
    }
    bool loadWeights(std::ifstream &fin) override{
        fin >> hparas.filterSize >> hparas.filterSize>>hparas.stride;
        //hparas.stride=2;
    }
    void showWeights() override {
        std::cout<<"hello"<<std::endl;
    }
    void showBias() override{;}
    std::vector<cv::Mat> compute(const std::vector<cv::Mat>& src,bool showInfo) override{

        std::vector<cv::Mat> temp=poolForward(src,hparas);
        //std::cout<<src[0];
        if(showInfo==1){
            std::cout<<"this is MaxPooling2D"<<std::endl;
            for(int i=0;i<src.size();i++){
                cv::Mat tmp=temp[i];
                std::cout<<"第"<<i<<"个："<<tmp.size<<"\n"<<tmp<<std::endl;
            }
        }
        return temp;
    }
};

class LayerFlatten:public layers{
private:
    std::string m_activation_type;
public:
    LayerFlatten(std::string type,int num)
            :layers(std::move(type),num){
    }
    bool loadWeights(std::ifstream &fin) override{
        //std::cout<<"Im fat"<<std::endl;
    }
    void showWeights() override {
        //std::cout<<"Im fat"<<std::endl;
    }
    void showBias() override{;}
    std::vector<cv::Mat> compute(const std::vector<cv::Mat>& src,bool showInfo) override{
        if(showInfo)
            std::cout<<"this is LayerFlatten"<<std::endl;
        std::vector<cv::Mat> temp;
        int row=src[0].rows;
        int col=src[0].cols;
        int size=src.size();
        //展开成一个向量
        //第几行第几列
        cv::Mat z=cv::Mat::zeros(1,row*col*size,CV_64F);
        auto* img = z.ptr<mytype >(0);
        for(int r=0;r<row;r++){
            for(int c=0;c<col;c++){
                //第几层
                for(int i=0;i<size;i++){
                    auto* it = src[i].ptr<mytype >(r);
                    *(img+i+size*c+col*size*r)=*(it+c);
                    //tmp.at<mytype >(0,0)=src[i].at<mytype >(r,c);
                    //temp.push_back(tmp.clone());
                    //std::cout<<src[i].at<mytype >(r,c)<<" ";
                }
            }
            //std::cout<<"第"<<i<<"个：\n"<<tmp.size<<std::endl;
        }
        temp.emplace_back(z);
        //std::cout<<std::endl;
        //std::cout<<temp.size()<<std::endl;
        return temp;
    }
};

class LayerDense:public layers{
private:
    std::string m_activation_type;
    cv::Mat wt;
    std::vector<mytype> bt;
public:
    LayerDense(std::string type,int num)
            :layers(std::move(type),num){
    }
    bool loadWeights(std::ifstream &fin) override{
        std::cout<<"I'm Dense"<<std::endl;
        fin >> hparas.input >> hparas.output;
        float tmp_float;
        char tmp_char = ' ';
        wt=cv::Mat::zeros(1,hparas.input,CV_64F);
        for(int i = 0; i < hparas.input; ++i) {
            fin >> tmp_char; // for '['
            for(int n = 0; n < hparas.output; ++n) {
                fin >> tmp_float;
                wt.at<mytype >(0,n+i*hparas.output)=tmp_float;
            }
            fin >> tmp_char; // for ']'
        }

        fin >> tmp_char; // for '['
        for(int n = 0; n < hparas.output; ++n) {
            fin >> tmp_float;
            bt.push_back(tmp_float);
        }
        fin >> tmp_char; // for ']'
    }
    void showWeights() override {
        std::cout<<"Im Dense"<<std::endl;
    }
    void showBias() override{;}
    std::vector<cv::Mat> compute(const std::vector<cv::Mat>& src,bool showInfo) override{
        if(showInfo)
            std::cout<<"this is Dense"<<std::endl;
        std::vector<cv::Mat> temp;
        cv::Mat results=cv::Mat::zeros(1,1,CV_64F);
        assert(src[0].size==wt.size);
        //第几行第几列
        //std::cout<<src[0]<<std::endl;
        mytype t=cv::sum(src[0].mul(wt))[0]+bt[0];
        results.at<mytype >(0,0)=t;
//        for(int i=0;i<src.size();i++){
//            //for(int j=0;j<W.size();j++){
//            mytype t=(src[i].at<mytype >(0,0))*W[i][0].at<mytype >(0,0);
//            results.at<mytype >(0,0)+=t;
//            //std::cout<<"第"<<i<<"个：\n"<<tmp.size<<std::endl;
//        }
        temp.push_back(results);
        return temp;
    }
};
//batchNormalization
class LayerBN:public layers{
private:
    mytype epslion;
    int depth;
    std::vector<std::vector<mytype>> params;
public:
    LayerBN(std::string type,int num)
            :layers(std::move(type),num){

    }
    bool loadWeights(std::ifstream &fin) override{
        char tmp_char = ' ';
        std::string tmp_str,m_border_mode;
        float tmp_float;
        bool skip = false;
        fin >> depth>> epslion;

        // reading kernel weights
        for(int k = 0; k < 4; ++k) {
            std::vector<mytype> temp;
            fin >> tmp_char; // for '['
            for(int d = 0; d < depth; ++d) {
                fin >> tmp_float;
                temp.push_back(tmp_float);
            }
            fin >> tmp_char; // for ']'
            params.push_back(temp);
        }

    }

    void showWeights() override {
        int filter=params.size();
        for(int i=0;i<filter;i++){
            std::cout<<"第"<<i<<"个:"<<std::endl;
            for(int j=0;j<depth;j++){
                std::cout<<params[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
    }
    //for scaling
    std::vector<cv::Mat> compute(const std::vector<cv::Mat>& src,bool showInfo) override{
        std::vector<mytype>& gamma=params[0];
        std::vector<mytype>& beta=params[1];
        std::vector<mytype>& mean=params[2];
        std::vector<mytype>& var=params[3];

        int size=src[0].rows;
        //std::cout<<src[0]<<std::endl;
        std::vector<cv::Mat> temp;
        for(int i=0;i<src.size();i++){
            cv::Mat tmp=cv::Mat::zeros(size,size,CV_64F);
            tmp=(src[i]-mean[i])/sqrt(var[i]+epslion);
            //std::cout<<mean[i]<<" \n"<<src[i]<<std::endl;
            tmp=gamma[i]*tmp+beta[i];
            temp.push_back(tmp.clone());
            if(showInfo==1){
                std::cout<<"this is LayerBN"<<std::endl;
                std::cout<<"第"<<i<<"个："<<tmp.size<<"\n";
                std::cout<<mean[i]<<" "<<var[i]<<" "<<gamma[i]<<" "<<beta[i]<<"\n";
                std::cout<<tmp<<std::endl;
            }
            ///std::cout<<tmp<<std::endl;
        }
        return temp;
    }
    void showBias() override{;
    }
};

namespace mycnn{
    std::vector<cv::Mat> data;
    std::vector<layers*> m_layers;
    bool loadAndPre(cv::Mat &img,cv::Mat &result);
    bool loadData(cv::Mat &img);
    bool loadWeights(const std::string &input_fname);
    mytype run();
}

//************************
//**loadAndPre
//**参数：图片地址，最终结果
//**功能：载入图片和预处理
//************************
bool mycnn::loadAndPre(cv::Mat &img,cv::Mat &result){
    //注意已经灰度化了
    //cv::Mat img =imread(address,cv::IMREAD_GRAYSCALE);
    //cout<<img.cols<<" "<<img.rows<<endl;
    if(img.cols==0)
        return false;
    //调整大小 同比缩放至fixedsize*fixedsize以内
    if(img.cols<img.rows)
        resize(img,img,{int(img.cols*1.0/img.rows*fixedSize),fixedSize});
    else
        resize(img,img,{fixedSize,int(img.rows*1.0/img.cols*fixedSize)});

    //剪去边上多余部分
    int cutRatio1=0.15*img.cols;
    int cutRatio2=0.05*img.rows;
    cv::Mat blank=cv::Mat(cv::Size(fixedSize,fixedSize), img.type(), cv::Scalar(0));//新建空白
    cv::Mat mask=img(cv::Rect(cutRatio1,cutRatio2,img.cols-2*cutRatio1,img.rows-2*cutRatio2));//建立腌摸
    cv::Mat imageROI=blank(cv::Rect(cutRatio1,cutRatio2,img.cols-2*cutRatio1,img.rows-2*cutRatio2));//建立需要覆盖区域的ROI
    mask.copyTo(imageROI, mask);

    //imshow("mask",mask);//小图
    //imshow("blank",blank);//大图

    int thre=getThreshold(blank);//均值获取阈值
    result=blank.clone();
    //补高光，而不直接粗暴二值化
    for (int i = 0; i<result.rows; i++){
        for (int j = 0; j<result.cols; j++){
            if((int)result.at<u_char>(i, j)>thre){
                result.at<u_char>(i, j)=200;
            }
        }
    }
    //imshow("result",result);
    //cv::waitKey();
    return true;
}
//************************
//**loadData
//**参数：图片地址
//**功能：载入图片和预处理
//************************
bool mycnn::loadData(cv::Mat &img){
    //清空一下
    mycnn::data.clear();
    cv::Mat image;//=cv::imread(input_fname,CV_BGR2GRAY);

    if(!loadAndPre(img,image))
        return false;
    cv::Mat tmp_single_layer=cv::Mat::zeros(fixedSize,fixedSize,CV_64F);
    for(int i=0;i<fixedSize;i++){
        for(int j=0;j<fixedSize;j++){
            tmp_single_layer.at<mytype >(i,j)=(double)image.at<uchar>(i,j);
        }
    }
    mycnn::data.push_back(tmp_single_layer);
    return true;
}

//************************
//**loadWeights
//**参数：参数文件地址
//**功能：读取文件中的层次信息并初始化集层
//************************
bool mycnn::loadWeights(const std::string &input_fname) {
    std::cout << "Reading model from " << input_fname <<std::endl;
    std::ifstream fin(input_fname.c_str());
    std::string layer_type;
    std::string tmp_str;
    int tmp_int = 0;
    int m_layers_cnt = 0;//层数
    fin >> tmp_str >> m_layers_cnt;//读入第一行 总层数 layers 层数
    std::cout << "Layers " << m_layers_cnt << std::endl;
    for(int layer = 0; layer < m_layers_cnt; ++layer) { // iterate over layers
        fin >> tmp_str >> tmp_int >> layer_type;//layer 层数 该层名称
        std::cout << "Layer " << tmp_int << " " << layer_type << std::endl;
        layers *l = nullptr;
        if(layer_type == "Convolution2D"||layer_type == "Conv2D") {
            l=new LayerConv2D(layer_type,tmp_int);
        }else if(layer_type == "Activation") {
            l = new LayerActivation(layer_type,tmp_int);
        }else if(layer_type == "MaxPooling2D") {
            l = new MaxPooling2D(layer_type,tmp_int);
        } else if(layer_type == "Flatten") {
            l = new LayerFlatten(layer_type,tmp_int);
        } else if(layer_type == "Dense") {
            l = new LayerDense(layer_type,tmp_int);
        } else if(layer_type == "Dropout"||layer_type == "InputLayer") {
            continue; // we dont need dropout layer in prediciton mode
        }else if(layer_type == "BatchNormalization"){
            l = new LayerBN(layer_type,tmp_int);
        }
        if(l == nullptr) {
            std::cout << "Layer is empty, maybe it is not defined? Cannot define network." << std::endl;
            return false;
        }
        l->loadWeights(fin);
        //l->showWeights();
        mycnn::m_layers.push_back(l);
    }
    fin.close();
    return true;
}

//************************
//**run
//**参数：
//**功能：计算最终结果
//************************
mytype mycnn::run(){
    std::vector<cv::Mat> tmp=mycnn::m_layers[0]->compute(mycnn::data);
    for(int i =1;i<mycnn::m_layers.size();i++){
        std::cout<<"第"<<i+1<<"层；来咯！:"<<std::endl;//<t.milli_cnt()<
        tmp=mycnn::m_layers[i]->compute(tmp);
    }
    mytype final=tmp[0].at<mytype >(0,0);
    return final;
}




