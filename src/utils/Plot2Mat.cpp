#include "Plot2Mat.h"

Plot2Mat::Plot2Mat() {
    im_width = 640;
    im_height = 480;
    bg_color = cv::Scalar(80,80,80);
    create( bg_color );
}

Plot2Mat::Plot2Mat(int _width, int _height, cv::Scalar _bg_color)
{
    im_width = _width;
    im_height = _height;
    bg_color = _bg_color;
    create( bg_color );
}


void Plot2Mat::resetCanvas() {
    create( bg_color );
}

void Plot2Mat::setYminmax( float _ymin, float _ymax ) {
    assert( _ymin<_ymax );
    ymin=_ymin;
    ymax=_ymax;
    setYminmax_dynamically = false;
}

void Plot2Mat::setYminmaxDynamic() {
    setYminmax_dynamically = true;
}

const cv::Mat& Plot2Mat::getCanvasConstPtr()
{
    return (const cv::Mat&) im;
}


void Plot2Mat::create( const cv::Scalar bg_color )
{
    this->im = cv::Mat( int(im_height*1.2), int(im_width*1.2), CV_8UC3, bg_color );
}



bool Plot2Mat::plot( const VectorXd& y, const cv::Scalar line_color, bool mark_plottedpts, bool label_plottedpts )
{
    // cout << "y.rows = " << y.rows() << endl;
    L = y.rows();
    assert( y.rows() > 0 );


    auto x = VectorXd( L );
    for( int i=0; i<L ; i++ ) x(i) = i;

    auto xd = x / L * im_width;


    if( setYminmax_dynamically ) {
    ymin = y.minCoeff(); //0.0;
    ymax = y.maxCoeff(); //1.0;
    }

    VectorXd yd = (y - ymin* VectorXd::Ones(L)) / (ymax-ymin) * im_height;

    // plot each point as circle
    for( int i=0 ; i<L ; i++ ) {
        // cout << "draw circle at " << xd(i) << "," << yd(i) << endl;
        if( mark_plottedpts || label_plottedpts ) {
            auto c = cv::Point2f( xd(i), -yd(i)+im_height );
            // plot the pt as circle
            if( mark_plottedpts )
                cv::circle( im, c, 6, cv::Scalar(0,0,255), -1 );

            // mark the text.
            if( label_plottedpts ) {
                char cordinate_pt_str[50];
                snprintf( cordinate_pt_str, 50, "%4.2f,%4.2f", x(i), y(i) );
                cv::putText(im, cordinate_pt_str, c, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,50,200), 1, CV_AA);
            }
        }

        auto c_xlabel = cv::Point2f( xd(i), im_height );
        if( i%int(L/5) == 0 )
        {
            cv::putText(im, std::to_string(i), cv::Point2f(c_xlabel.x,c_xlabel.y+20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);

            cv::circle( im, c_xlabel, 4, cv::Scalar(255,255,255), -1 );
            cv::line( im, c_xlabel, cv::Point2f(c_xlabel.x, 0), cv::Scalar(255,255,255), 2 );
        }
    }


    // lines between pt[i-1] <----> pt[i]
    for( int i=1 ; i<L ; i++ ) {
        auto p1 = cv::Point2f( xd(i), -yd(i) + im_height );
        auto p2 = cv::Point2f( xd(i-1), -yd(i-1) + im_height );
        cv::line( im, p1, p2, line_color, 2 );
    }


    // ylabels
    double step = (ymax - ymin)/10.;
    for( double _yd = ymin ; _yd<=ymax ; _yd+=step )
    {
        double _y = (_yd - ymin) / (ymax-ymin) * im_height;
        _y = -_y + im_height;
        cv::line( im, cv::Point2f(0., _y), cv::Point2f(im_width, _y), cv::Scalar(255,255,255), 2 );

        char buf[15];
        snprintf( buf, 15, "%.2f", _yd );
        cv::putText(im, buf, cv::Point2f(im_width+20,_y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);
    }
}




bool Plot2Mat::mark( const int x_, const cv::Scalar color, bool enable_text_label )
{
    cout << "Plot2Mat::mark(" << x_ << ")\n";
    assert( L>0 && "you called mark before plotting. the intended use is to call plot and then optionally call mark" );
    float xd = float(x_) / float(L) * im_width;
    auto c = cv::Point2f( xd, im_height );
    cv::circle( im, c, 7, color, -1 );

    if( enable_text_label )
        cv::putText(im, to_string(x_), c, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);

}
