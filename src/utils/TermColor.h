#pragma once

/**

// TODO Work in progress
// https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal

black        30         40
red          31         41
green        32         42
yellow       33         43
blue         34         44
magenta      35         45
cyan         36         46
white        37         47

reset             0  (everything back to normal)
bold/bright       1  (often a brighter shade of the same colour)
underline         4
inverse           7  (swap foreground and background colours)
bold/bright off  21
underline off    24
inverse off      27

eg:
cout << "\033[1;31mbold red text\033[0m\n";



*/
#include <iostream>
#include <sstream>
#include <string>

class TermColor
{
public:
    static std::string RED() { return compose( TermColor::BG_RED );  }
    static std::string GREEN() { return compose( TermColor::BG_GREEN );  }
    static std::string YELLOW() { return compose( TermColor::BG_YELLOW );  }
    static std::string BLUE() { return compose( TermColor::BG_BLUE );  }
    static std::string MAGENTA() { return compose( TermColor::BG_MAGENTA );  }
    static std::string CYAN() { return compose( TermColor::BG_CYAN );  }
    static std::string WHITE() { return compose( TermColor::BG_WHITE );  }

    static std::string iRED() { return compose( TermColor::BG_RED, true );  }
    static std::string iGREEN() { return compose( TermColor::BG_GREEN, true );  }
    static std::string iYELLOW() { return compose( TermColor::BG_YELLOW, true );  }
    static std::string iBLUE() { return compose( TermColor::BG_BLUE, true );  }
    static std::string iMAGENTA() { return compose( TermColor::BG_MAGENTA, true );  }
    static std::string iCYAN() { return compose( TermColor::BG_CYAN, true );  }
    static std::string iWHITE() { return compose( TermColor::BG_WHITE, true );  }

    static std::string RESET()
    {
        std::stringstream buffer;
        buffer << "\033[" << CTRL_RESET << "m";
        return buffer.str();
    }

    // modifier : bold, underline etc.
    // color : FG_RED, ..., BG_RED etc
    static std::string compose( const int modifier, const int color, bool invert=false )
    {

        std::stringstream buffer;
        if( !invert )
            buffer << "\033[" << modifier << ";" << color << "m";
        else
            buffer << "\033[" << CTRL_INVERSE << ";" << modifier << ";" << color << "m";
        return buffer.str();
    }

    static std::string compose( const int color, bool invert=false )
    {
        std::stringstream buffer;
        if( !invert )
            buffer << "\033[" << color << "m";
        else
            buffer << "\033[" << CTRL_INVERSE << ";" << color << "m";
        return buffer.str();
    }

    static const int BG_BLACK = 30;
    static const int BG_RED = 31;
    static const int BG_GREEN = 32;
    static const int BG_YELLOW = 33;
    static const int BG_BLUE = 34;
    static const int BG_MAGENTA = 35;
    static const int BG_CYAN = 36;
    static const int BG_WHITE = 37;

    static const int FG_BLACK = 40;
    static const int FG_RED = 41;
    static const int FG_GREEN = 42;
    static const int FG_YELLOW = 43;
    static const int FG_BLUE = 44;
    static const int FG_MAGENTA = 45;
    static const int FG_CYAN = 46;
    static const int FG_WHITE = 47;

    static const int CTRL_RESET = 0;
    static const int CTRL_BOLD = 1;
    static const int CTRL_UNDERLINE = 4;
    static const int CTRL_INVERSE = 7; //(swap foreground and background colours)
    static const int CTRL_BOLD_OFF = 21;
    static const int CTRL_UNDERLINE_OFF = 24;
    static const int CTRL_INVERSE_OFF = 27; //(swap foreground and background colours)
};
