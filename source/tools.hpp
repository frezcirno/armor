//
// Created by truth on 2020/3/8.
//tools.hpp includes :
//some functions for detailed implimentations and pretreatments
//hpp acquired
//basic structs statements

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
const int fixedSize=32;
#ifndef MYCNN_TOOLS_HPP
#define MYCNN_TOOLS_HPP
#endif //MYCNN_TOOLS_HPP
typedef double mytype;
struct hparameters{
    int stride;
    int pad;
    int filterSize;
    int input;
    int output;
};
enum poolType{
    max=0,
    aver
};
//************************
//**relu
//**激活函数
//**功能： output x>0? x:0
//************************
cv::Mat relu(const cv::Mat& x) {
    //cout<<x<<endl;
    cv::Mat a=cv::Mat::zeros(x.rows,x.cols,CV_64F);
    int i,j;
    for ( i = 0; i<a.rows; i++){
        for ( j = 0; j<a.cols; j++){
            a.at<mytype >(i, j)=(0<x.at<mytype >(i,j)?x.at<mytype >(i,j):0);
        }
    }
    //std::cout<<"a"<<std::endl;
    //std::cout<<a<<std::endl;
    return a;
}

//************************
//**sigmoid
//**激活函数
//**功能： output 1/(1+exp(-x))
//************************
cv::Mat  sigmoid(const cv::Mat& x) {
    //cout<<x<<endl;
    cv::Mat a=cv::Mat::zeros(x.rows,x.cols,CV_64F);
    int i,j;
    for ( i = 0; i<a.rows; i++){
        for ( j = 0; j<a.cols; j++){
            a.at<mytype >(i, j)=1/(1+exp(-x.at<mytype >(i, j)));
        }
    }
    return a;
}

//************************
//**getThreshold
//**图片取得阈值
//**参数：一目了然
//**功能：获得前百分比高光
//************************
int getThreshold(const cv::Mat& mat,double thre_proportion=0.1){

    uint32_t iter_rows = mat.rows;
    uint32_t iter_cols = mat.cols;
    auto sum_pixel = iter_rows * iter_cols;
    if(mat.isContinuous()){
        iter_cols = sum_pixel;
        iter_rows = 1;
    }
    int histogram[256];
    memset(histogram, 0, sizeof(histogram));//置零
    for (uint32_t i = 0; i < iter_rows; ++i){
        const auto* lhs = mat.ptr<uchar>(i);
        for (uint32_t j = 0; j < iter_cols; ++j)
            ++histogram[*lhs++];
    }

    auto left = thre_proportion * sum_pixel;
    int i = 255;
    while((left -= histogram[i--]) > 0);
    return i>0?i:0;

}



//************************
//**zeroPadding
//**参数：输入图片，输出图片，拓展宽度
//**功能： 扩宽以补充卷积带来的损失
//************************
bool zeroPadding(const std::vector<cv::Mat>& src, std::vector<cv::Mat>& dst,const int& pad){
    //错误提醒
    if( src.empty() ){
        printf(" No data entered, please enter the path to an image file \n");
        return false;
    }
    //上下左右的拓展宽度
    int top,left,bottom,right;
    top=left=bottom=right=pad;
    //copyMakeBorder的参数 拓展值为常数
    int borderType = cv::BORDER_CONSTANT;
    cv::Scalar value=cv::Scalar(0);
    //拓展黑
    for_each(src.begin(), src.end(), [&]
            (const cv::Mat& i) {
        cv::Mat j;
        cv::copyMakeBorder( i, j, top, bottom, left, right, borderType, value );
        dst.push_back(j);
    });
    return true;
}

//************************
//**convSingleStep
//**参数：in>输入像素块，输入w
//**     out<卷积后的新像素值
//**功能： conv最小单元（未用到，考虑提速）
//************************
mytype convSingleStep(const cv::Mat& aSlicePre,const cv::Mat& W){
    int size=W.rows;
    mytype result=0;
    //以下操作：result=sum(aSlicePre.*w)
    for (int i = 0; i < size; ++i){
        const auto* img = aSlicePre.ptr<mytype >(i);
        const auto* w = W.ptr<mytype >(i);
        for (int j = 0; j < size; ++j)
            result+=(*(img+j))*(*(w+j));
    }
    return result;
}
/*A_prev 一维（仅限灰度化后的） 层数
 *W 、 b   二维  第几个filter 第几层 层数要和A_pre一致
 *
 * */
//************************
//**convForward
//**参数：in>输入图片（几层depth）【注】：改代码需要转为灰度化使用
//**        输入w、b(第几个filter)
//**        输入hparas（决定了步长等信息）
//**     return< conv后的新图片层
//**功能： conv功能实现
//************************
std::vector<cv::Mat> convForward(const std::vector<cv::Mat>& A_prev,
                                 const std::vector<std::vector<cv::Mat>>& W,
                                 const std::vector<mytype >& b,
                                 const hparameters& hparas){
//判断输入图片深度和w深度是否一致
    if(A_prev.size()==W[0].size())
        std::cout<<"skr"<<std::endl;
    else std::cout<<"fuck"<<std::endl;
    assert(A_prev.size()==W[0].size());

    std::vector<cv::Mat>result;
    //对图像来说
    int n_H_prev{A_prev[0].rows},n_W_prev{A_prev[0].cols},
            filterSize{W[0][0].rows};//
    //输出的层数取决有几个filter
    int n_C=W.size();
    int &f=filterSize;
    int pad=hparas.pad,stride=hparas.stride;
//计算卷积后的图片大小
    int n_H = 1 + int((n_H_prev + 2 * pad - f) / stride);
    int n_W = 1 + int((n_W_prev + 2 * pad - f) / stride);
    int n_D=A_prev.size();

    //padding
    std::vector<cv::Mat>A_prev_pad;
    if(hparas.pad==0){
        A_prev_pad=A_prev;
    } else
        zeroPadding(A_prev, A_prev_pad,pad);

    int c,h,w,i,j,x;

    int total = 0,small=0;
    int xx=(f)*(f)*n_D;
    int xxx=n_W*n_H;
//把所有的w按照像素块的模式存储好
//n_C：第几个filter输出 xx：某一个filter中的w从mat展开成一个向量
    mytype oneW[n_C][xx];
    memset(oneW, 0, sizeof(oneW));//置零
    for(c =0;c<n_C;c++){
        for(  i =0,small=0;i<n_D;++i ){//depth
            for ( x = 0; x < f; ++x){//行
                const auto* wt = W[c][i].ptr<mytype >(x);
                for ( j = 0; j < f; ++j,small++){//列
                    oneW[c][small]=(*(wt+j));//temp.push_back(*(wt+j));
                }
            }
        }
    }
//把所有的图片按不同层的像素块存储好
//n_C：第几个filter输出 xx：某一个filter中的w从mat展开成一个向量
    mytype onePixel[xxx][xx];
    memset(onePixel, 0, sizeof(onePixel));//置零

//具体到某一幅图像
    for ( h=0,total=0;h<n_H;++h){
        for ( w=0;w<n_W ;++w,total++){
//具体到某一图像的某一深度
//确定待卷积像素块的位置
            int vert_start = h * stride;
            int horiz_start = w * stride;
            //按层次展开
            for(  i =0,small=0;i<n_D;++i ){
                for ( x = 0; x < f; ++x){
                    //找一下原图上的待卷积像素块
                    const auto* it = A_prev_pad[i].ptr<mytype >(vert_start+x);//第几列
                    for ( j = 0; j < f; ++j,++small){
                        onePixel[total][small]=(*(it+horiz_start+j));
                    }
                }
            }
        }
    }
//循环所有层次
    for ( c =0;c<n_C;c++) {
//具体到某一幅图像
        cv::Mat z = cv::Mat::zeros(n_H,n_W,CV_64F);
        //逐个像素点确定
        for ( h=0;h<n_H;++h){
            auto* img = z.ptr<mytype >(h);
            for ( w=0;w<n_W ;++w){
                mytype t=0;
                for(i=0;i<f*f*n_D;i+=3){
                    //单纯为了加快速度
                    t+=onePixel[w+n_W*h][i]*oneW[c][i];
                    t+=onePixel[w+n_W*h][i+1]*oneW[c][i+1];
                    t+=onePixel[w+n_W*h][i+2]*oneW[c][i+2];
                }
                *(img+w)=t;//+b[c];//b[c][0].at<mytype >(0,0);
            }
        }
        z+=b[c];
        cv::Mat temp=z.clone();
        //std::cout<<"第"<<c<<"个:"<<temp.size<<"\n"<<temp<<std::endl;
        result.emplace_back(temp);
    }

    return result;
}

//************************
//**poolForward
//**参数：in>输入图片（几层depth）
//**        输入hparas（决定了步长等信息）
//**        输入mode(决定模式)
//**     return<pooling后的新图片层
//**功能： pooling功能实现
//************************
std::vector<cv::Mat> poolForward(const std::vector<cv::Mat>& A_prev,
                                 const hparameters& hparas,int mode=max){
    std::vector<cv::Mat> result;
    int n_H_prev{A_prev[0].rows},n_W_prev{A_prev[0].cols};
    int f=hparas.filterSize;
    int stride=hparas.stride;
//确定pooling后的图像大小
    int n_H = 1 + int((n_H_prev  - f) / stride);
    int n_W = 1 + int((n_W_prev  - f) / stride);
    //int count=0;
    //逐层遍历
    for(const auto & i : A_prev){
        cv::Mat z = cv::Mat::zeros(n_H,n_W,CV_64F);
        //逐点遍历
        for (int h=0;h<n_H;h++){
            auto* img = z.ptr<mytype >(h);
            for (int w=0;w<n_W ;w++){
                //确定pooling像素块起始点
                int vert_start = h * stride;
                int horiz_start = w * stride;
                //maxpooling or averpooling
                if (mode == max){
                    mytype maxVal=0;
//filter内寻找最大值代替
                    for ( int x = 0; x < f; ++x){
                        const auto* it = i.ptr<mytype >(vert_start+x);//第几列
                        for ( int j = 0; j < f; ++j){
                            if(*(it+horiz_start+j)>maxVal)
                                maxVal=*(it+horiz_start+j);
                            //count++;
                        }
                    }
                    *(img+w)=maxVal;
                }
                else if(mode == aver){
                    ;//待补充
                }
            }
        }
        cv::Mat temp=z.clone();
        result.push_back(z);
    }
    //std::cout<<count<<std::endl;
    return result;
}