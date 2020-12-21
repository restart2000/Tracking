#include<stdlib.h>
#include<cv.h>
#include<highgui.h>
#include <vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include"ros/ros.h"
#include"std_msgs/String.h"
#include"std_msgs/Bool.h"
#include"std_msgs/Float32.h"

#include<geometry_msgs/Twist.h>
#include"sensor_msgs/Image.h"

#define LINEAR_X 0

#define pi 3.1415926

using namespace cv;
using namespace std;


int center_x = 0;
int center_y = 0;
int erea_max = 0;
int threshold_forward = 50000;//5w
int threshold_stop = 10000;//1w
int threshold_start = 100000;//10w
int flag = 0;
int threshold_turn = 30;
double k_speed = 1.0;
double k_turn = 0.004;

void test_erea(Mat binary)
{
    Mat labels;
    Mat stats;
    Mat centroids;

    int nccomps = cv::connectedComponentsWithStats (
            binary, //二值图像
            labels,     //和原图一样大的标记图
            stats, //nccomps×5的矩阵 表示每个连通区域的外接矩形和面积（pixel）
            centroids //nccomps×2的矩阵 表示每个连通区域的质心
            );
   // cout<<nccomps<<endl;//区域数量

    vector<cv::Vec3b> colors(nccomps+1);

    colors[0] = Vec3b(0,0,0); // background pixels remain black.


    erea_max = 0;
    int erea_max_i = 1;//从1开始，0是背景 
    erea_max = stats.at<int>(erea_max_i, cv::CC_STAT_AREA);

    
    
    for(int i = 2; i < nccomps; i++ )//找最大面积
    {
        
        if( erea_max < stats.at<int>(i, cv::CC_STAT_AREA) )
        {
            erea_max = stats.at<int>(i, cv::CC_STAT_AREA);
            erea_max_i = i;
            
        }
        
    }
    //cout<<erea_max<<endl;
    


    int erea_threshold = erea_max*0.8;

    //画彩色图
    for(int i = 1; i < nccomps; i++ )
    {
        colors[i] = Vec3b(rand()%256, rand()%256, rand()%256);

        if( stats.at<int>(i, cv::CC_STAT_AREA) < erea_threshold )
        {
            colors[i] = Vec3b(0,0,0); //不画小于阈值的区域
        }   
    }

    Mat img_color = Mat::zeros(binary.size(), CV_8UC3);

    for( int y = 0; y < img_color.rows; y++ )
    {
        for( int x = 0; x < img_color.cols; x++ )

        {

            int label = labels.at<int>(y, x);

            CV_Assert(0 <= label && label <= nccomps);//检测是否出错

            img_color.at<cv::Vec3b>(y, x) = colors[label];

        }
    }

    namedWindow("test_erea",0);//创建窗口
    cvResizeWindow("test_erea", 500, 500); //创建一个500*500大小的窗口
    imshow("test_erea",img_color);
    //画图结束

    //求质心
    cout<<erea_max;
    center_x = centroids.at<double>(erea_max_i,0);//注意类型double！！
    center_y = centroids.at<double>(erea_max_i,1);

}


Mat image_process(Mat input)
{
    Mat output;
    Mat temp;
    Mat gray;
    cvtColor(input,gray,COLOR_BGR2GRAY);
    
    //imshow("gray",gray);
    GaussianBlur(gray, gray, Size(5, 5), 4, 4);//除噪声
   

    //二值化处理、大津法
    Mat binary; 
    //threshold(gray,binary,0,255,CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    threshold(gray,binary,30,255,CV_THRESH_BINARY_INV);

    //腐蚀
    //获取自定义核
	Mat element = getStructuringElement(MORPH_RECT, Size(30, 30));//大小可能要调 
    erode(binary,binary,element);

    test_erea(binary);

    output = binary.clone();
    return output;
   
}

int main(int argc,char **argv)
{

    VideoCapture capture;
    capture.open(0);

    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");
    ros::NodeHandle n;

    //ros::Rate loop_rate(10);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel",5);

    if(!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上的摄像头\n");
       // return 0;
    }
    waitKey(1000);
    Mat frame;//当前帧图片
    int nFrames = 0;//当前帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽度
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高度

    while(ros::ok())
    {
        capture.read(frame);
        if(frame.empty())
        {
            //break;
        }
    //imshow("frame",frame);
    Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
    
    //Mat frIn = imread("/home/howard/expt_3.JPG");
    


    Mat processed;
    processed=image_process(frIn);
    namedWindow("processed",0);//创建窗口
    cvResizeWindow("processed", 500, 500); //创建一个500*500大小的窗口
    imshow("processed",processed);
    

    Point p(center_x,center_y);
    cout<<p<<endl;
	circle(frIn, p, 10, Scalar(0, 0, 255),-1);
    Point target(frIn.cols/2,frIn.rows/2);
    circle(frIn, target, 10, Scalar(255, 0, 0),-1); 
    namedWindow("frIn",0);//创建窗口
    cvResizeWindow("frIn", 600, 600); //创建一个600*600大小的窗口
    imshow("frIn",frIn);

    //cout<<frIn.rows<<' '<<frIn.cols<<endl;
    
    geometry_msgs::Twist cmd_red;
	cmd_red.linear.x = 0;
	cmd_red.linear.y = 0;
	cmd_red.linear.z = 0;
	cmd_red.angular.x = 0;
	cmd_red.angular.y = 0;
	cmd_red.angular.z = 0;

    if(erea_max>threshold_start )
    {
        flag = 1;
    }
    
    if(erea_max<threshold_stop && flag)
    {
        flag = 0;
    }

   

    if(flag)
    {

	int delta = center_x - frIn.cols/2;
	long int delta_area = threshold_forward - erea_max;
	
        if(erea_max<threshold_forward)
        {
            cmd_red.linear.x = k_speed*delta_area/threshold_forward;
	        cout<<cmd_red.linear.x<<"x"<<endl;
	    
        }   
	    if(abs(delta) > threshold_turn)
        {
            cmd_red.angular.z = -delta * k_turn;//逆时针为正
	    if(delta>frIn.cols/2*0.67)
	    {
		    cmd_red.angular.z = 1.2*cmd_red.angular.z;
            cout<<cmd_red.linear.x<<"zzz"<<endl;
	    }
        }

        
        

    }


	pub.publish(cmd_red);

    ros::spinOnce();
    waitKey(5);
    }

}
    
