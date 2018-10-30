#include <iostream>

#include "../utils/TermColor.h"
// #include "../utils/PoseManipUtils.h"

using namespace std;
int main()
{
    cout << "Hello World\n";
    cout << TermColor::RED() << "Hello World in red" << TermColor::RESET() << endl;
    cout << TermColor::iRED() << "Hello World in red" << TermColor::RESET() << endl;


    cout << TermColor::compose( TermColor::CTRL_BOLD, TermColor::FG_RED, true ) << "Hello" << TermColor::RESET() << endl;
    cout << TermColor::compose( TermColor::CTRL_BOLD, TermColor::FG_RED, false ) << "Hello" << TermColor::RESET() << endl;
    cout << TermColor::compose( TermColor::FG_BLUE, true ) << "Hello" << TermColor::RESET() << endl;
    cout << TermColor::compose( TermColor::FG_BLUE, false ) << "Hello" << TermColor::RESET() << endl;

    cout << TermColor::compose( TermColor::BG_CYAN ) << "Hello" << TermColor::RESET() << endl;
    cout << TermColor::compose( TermColor::CTRL_UNDERLINE, TermColor::BG_GREEN ) << "Hello" << TermColor::RESET() << endl;
    return 0;
}
