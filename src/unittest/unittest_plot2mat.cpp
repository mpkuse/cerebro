#include <iostream>
using namespace std;

#include "../utils/Plot2Mat.h"
int demo()
{
    cout << "Hello\n";
    Plot2Mat han;
    han.setYminmax( -2, 2 );

    for( int i=10; i<50; i++ ) {
    VectorXd y = VectorXd::Random(10*i);
    cout << "y=" << y.transpose() << endl;

    han.mark( i );

    han.plot(y);
    cv::imshow("canvas", han.getCanvasConstPtr() );
    cv::waitKey(0);

    han.resetCanvas();
    }

    cout << "Finished...!\n";
}


int demo_easy() {
    cout <<"demo_easy\n";

    Plot2Mat han;
    han.setYminmax( -2, 2 );

    VectorXd y = VectorXd::Random(10);
    cout << "y=" << y.transpose() << endl;

    han.mark( 4 );
    han.plot(y, cv::Scalar(0,255,0), true, true);
    // han.plot(y);
    cv::imshow("canvas", han.getCanvasConstPtr() );
    cv::waitKey(0);


}

int main() {
    demo_easy();
    // demo();
}
