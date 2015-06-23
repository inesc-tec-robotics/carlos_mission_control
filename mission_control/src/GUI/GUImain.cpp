/***************************************************************************
 * Software License Agreement (BSD License)                                *
 *                                                                         *
 *  Copyright (c) 2012, Mikkel Hvilshoj, Christian Caroe, Casper Schou     *
 *	Department of Mechanical and Manufacturing Engineering             *
 *  Aalborg University, Denmark                                            *
 *  All rights reserved.                                                   *
 *                                                                         *
 *  Redistribution and use in source and binary forms, with or without     *
 *  modification, are permitted provided that the following conditions     *
 *  are met:                                                               *
 *                                                                         *
 *  - Redistributions of source code must retain the above copyright       *
 *     notice, this list of conditions and the following disclaimer.       *
 *  - Redistributions in binary form must reproduce the above              *
 *     copyright notice, this list of conditions and the following         *
 *     disclaimer in the documentation and/or other materials provided     *
 *     with the distribution.                                              *
 *  - Neither the name of Aalborg University nor the names of              *
 *     its contributors may be used to endorse or promote products derived *
 *     from this software without specific prior written permission.       *
 *                                                                         *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    *
 *  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS      *
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE         *
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    *
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,   *
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;       *
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT     *
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN      *
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        *
 *  POSSIBILITY OF SUCH DAMAGE.                                            *
 ***************************************************************************
 *
 * This is the first window in the GUI. The grapical layout is created in the ui-header file
 * "ui_MainWindow.h".
 *
 * The MainWindow contains three buttons which connects to Program.cpp, Execute.cpp,
 * WorkSpace.cpp, RobotControl.cpp, and Credits.cpp.
 *
 * Theese windows are kept open untill they are either accepted or rejected. The last button
 * Exit terminates the GUI. This is only possible if there is no other windows running.
 *
 */
#include "GUImain.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <signal.h>
#include <cstdlib>

using namespace std;

GUImain::GUImain(int argc, char *argv[])
{
    GUIthread = boost::thread(&GUImain::runGUIthread, this, argc, argv);
}

GUImain::~GUImain()
{
    GUIthread.interrupt();
    GUIthread.join();

}

void GUImain::runGUIthread(int argc, char *argv[])
{
    try
    {

        QApplication a(argc, argv);

        MainWindow w;

        w.show();

        a.exec();
    }
    catch (std::exception)
    {
        cout << "std::exeption" << endl;
    }
    catch (int x)
    {
        cout << "some other exception" << endl;
    }

    raise(SIGINT);
}



