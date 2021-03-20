/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_BATTERY_UPDATE 20
#define PRIORITY_TREFRESHWD 99
#define PRIORITY_ACTION_CAMERA 19
#define PRIORITY_COM_CAMERA 23

// CONSANTES LOCALES POUR LES COMMANDES DE LA CAMERA
#define CAMERA_ASK_ARENA 300
#define CAMERA_STREAM   200
#define CAMERA_FIND_POSITION 400

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !

 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_commandCamera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_watchDog, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_error_count, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_period, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_actionType, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arenaConfirmed, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_CamCommunication, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_refreshWatchDog, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_sem_create(&sem_start_Stream, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_sem_create(&sem_arenaResult, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_check_battery_level, "th_battery_update", 0, PRIORITY_BATTERY_UPDATE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_refreshWatchDog, "th_refresh_watch_Dog", 0, PRIORITY_TREFRESHWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_actionProcess_Camera, "th_action_Camera_Process", 0, PRIORITY_ACTION_CAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_comCamera, "th_com_camera", 0, PRIORITY_COM_CAMERA, 0)) {
        cerr << "Error task create: (th_comCamera) " << strerror(-err) << endl << flush;
        
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if(err = rt_task_start(&th_check_battery_level,(void(*)(void*) ) & Tasks::UpdateBatteryLevel,this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    } 
    if(err = rt_task_start(&th_refreshWatchDog,(void(*)(void*) ) & Tasks::RefreshWatchDog,this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // DONE
    // FINISHED
    if(err = rt_task_start(&th_comCamera,(void(*)(void*) ) & Tasks::CamCommunicationTask,this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // DONE
    if(err = rt_task_start(&th_action_Camera_Process,(void(*)(void*) ) & Tasks::ActionCameraHandler,this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }


    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        cout << "mutex acquire " << endl << flush;
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
        cout << "mutex released " << endl << flush;
        cout << "Message sent: "<< endl << flush;
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            // Signaler qu'on demarre sans le mode wd

            rt_mutex_acquire(&mutex_watchDog,TM_INFINITE);
            watchDog = false;
            rt_mutex_release(&mutex_watchDog);
            rt_sem_v(&sem_startRobot);
        }else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            // Signaler qu'on demarre avec le mode wd

            rt_mutex_acquire(&mutex_watchDog,TM_INFINITE);
            watchDog = true;
            rt_mutex_release(&mutex_watchDog);

            rt_sem_v(&sem_startRobot);
            rt_sem_v(&sem_refreshWatchDog); 
        }
        
         else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }else if(msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA) ||
                msgRcv->CompareID(MESSAGE_CAM_OPEN) ||
                msgRcv->CompareID(MESSAGE_CAM_CLOSE) ||
                msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM) ||
                msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM) ||
                msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START) ||
                msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP))

            {
            rt_mutex_acquire(&mutex_commandCamera,TM_INFINITE);
            commandCamera = msgRcv->GetID();
            rt_mutex_release(&mutex_commandCamera);
            rt_sem_v(&sem_CamCommunication);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    bool wd;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        // Start Robot
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        // get mode 
        rt_mutex_acquire(&mutex_watchDog,TM_INFINITE);
        wd = watchDog;
        rt_mutex_release(&mutex_watchDog);
        if(wd){
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_sem_v(&sem_refreshWatchDog);
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
        }else{
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
        }
        
        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    Message* res;
    int cpt;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        //cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            res = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);


            if(!res->CompareID(MESSAGE_ANSWER_ACK)){
                rt_mutex_acquire(&mutex_error_count,TM_INFINITE);
                error_count++;
                cpt = error_count;
                rt_mutex_release(&mutex_error_count);
                cout << "Communication error" << endl << flush;
                if(cpt>=3){
                    cout<< "Restart" << endl << flush;
                    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                    robotStarted=0;
                    rt_mutex_release(&mutex_robotStarted);
                    Message* m= new Message(MESSAGE_ANSWER_COM_ERROR);
                    WriteInQueue(&q_messageToMon,m);
                    
                    rt_mutex_acquire(&mutex_robot,TM_INFINITE);
                    
                    robot.Close();
                    cout<< "Robot closed" << endl << flush;
                    //robot.Reset();
                    rt_mutex_release(&mutex_robot);
                    
                }
            }else{
                rt_mutex_acquire(&mutex_error_count,TM_INFINITE);
                error_count = 0;
                rt_mutex_release(&mutex_error_count);
            }
        }
        //cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}



// Nos taches
void Tasks::UpdateBatteryLevel(void* arg){
   //Variables
    Message * msgSend;
    int rs; // reponse du rebot
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    //Synchronization barrier, awaiting for all the tasks to start
    rt_sem_p(&sem_barrier, TM_INFINITE);
    //rt_sem_p(&sem_batteryLevel, TM_INFINITE);
    
    //Task
    rt_task_set_periodic(&th_check_battery_level, TM_NOW, rt_timer_ns2ticks(50000000));
    
    cout << "Battery started" << endl;
    
    while (1) {
        //cout << "Battery started while" << endl;
        rt_task_wait_period(NULL);
        
        //Verify if the Robot is active
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if (rs) {
           cout << "Update battery" << endl << flush;
           
           //Get the battery level update
           rt_mutex_acquire(&mutex_robot, TM_INFINITE);
           msgSend = robot.Write(robot.GetBattery());
           rt_mutex_release(&mutex_robot);
            
           if(msgSend != NULL && !msgSend->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT))
           {
                if((dynamic_cast<MessageBattery*>(msgSend))->GetLevel() == BATTERY_EMPTY){
                   cout << "Update : Empty Battery \n" << endl;
                }else if((dynamic_cast<MessageBattery*>(msgSend))->GetLevel() == BATTERY_LOW){
                   cout << "Update : Low Battery \n" << endl;

                }else if((dynamic_cast<MessageBattery*>(msgSend))->GetLevel() == BATTERY_FULL){
                   cout << "Update : Full Battery \n" << endl;
                }else{
                   cout << "Update : Unknown Battery Level \n" << endl;
                }
               //Send the battery level update to the monitor
                WriteInQueue(&q_messageToMon, msgSend);

           } else {
               cout << "Response unknown \n" << endl;
           }
        }
    }
}

void Tasks::ActionCameraHandler(void* arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    rt_sem_p(&sem_barrier,TM_INFINITE);
    rt_sem_p(&sem_start_Stream,TM_INFINITE); // attendre que la camera soit ourverte
    // lire la fréquence d'échantillonage

    rt_mutex_acquire(&mutex_period,TM_INFINITE);
    rt_task_set_periodic(NULL,TM_NOW,period);
    rt_task_release(&mutex_period);

    bool auxCanStream;
    int auxTypeAction;
    Arena auxArena;

    rt_mutex_acquire(&mutex_camera,TM_INFINITE);
    Img img = camera.Grab();
    rt_mutex_release(&mutex_camera);

    rt_mutex_acquire(&mutex_camera,TM_INFINITE);
    Img imgArena = camera.Grab();
    rt_mutex_release(&mutex_camera);

    bool auxArenaConfirmed;
    Position position;

    while(1){
        rt_mutex_acquire(&mutex_continueStream,TM_INFINITE);
        auxCanStream = canStream;
        rt_mutex_release(&mutex_continueStream);
        if(auxCanStream){
            rt_mutex_acquire(&mutex_actionType,TM_INFINITE);
            auxTypeAction = actionCamera;
            rt_mutex_release(&mutex_actionType);

            switch(auxTypeAction){
                case CAMERA_STREAM :
                    rt_task_wait_period(NULL);
                    rt_mutex_acquire(&mutex_camera,TM_INFINITE);
                    img = camera.Grab();
                    rt_mutex_release(&mutex_camera);

                    // send image to
                    MessageImg* pImageTosend = new MessageImg(MESSAGE_CAM_IMAGE,&img);
                    WriteInQueue(&q_messageToMon,pImageTosend);

                    break;
                case CAMERA_ASK_ARENA:
                    rt_task_set_periodic(NULL,TM_NOW,0);
                    rt_mutex_acquire(&mutex_camera,TM_INFINITE);
                    imgArena = camera.Grab(); // recadrage de l'arena
                    rt_mutex_release(&mutex_camera);

                    rt_mutex_acquire(&mutex_arena,TM_INFINITE);
                    arena = imgArena.SearchArena();
                    auxArena = arena;
                    rt_mutex_release(&mutex_arena);

                    if(auxArena.IsEmpty()){
                        WriteInQueue(&q_messageToMon,new Message(MESSAGE_ANSWER_NACK));
                        rt_mutex_acquire(&mutex_actionType,TM_INFINITE);
                        actionCamera = CAMERA_STREAM; // continue to stream
                        rt_mutex_release(&mutex_actionType);
                    }else{
                        imgArena.DrawArena(auxArena);
                        WriteInQueue(&q_messageToMon,new MessageImg(MESSAGE_CAM_IMAGE,&imgArena));

                        //rt_sem_p(&sem_arenaResult,TM_INFINITE);

                        rt_mutex_acquire(&mutex_arenaConfirmed,TM_INFINITE);
                        auxArenaConfirmed = arenaConfirmed;
                        rt_mutex_release(&mutex_arenaConfirmed);

                        if(auxArenaConfirmed){
                            cout << "Arean saved " << end << flush;
                        }

                        rt_task_set_periodic(NULL,TM_NOW,period);
                        rt_mutex_acquire(&mutex_actionType,TM_INFINITE);
                        actionType = CAMERA_STREAM;
                        rt_mutex_release(&mutex_actionType);
                    }

                    break;
                case CAMERA_FIND_POSITION:
                    rt_task_set_periodic(NULL,TM_NOW,0);
                    rt_mutex_acquire(&mutex_arena,TM_INFINITE);
                    auxArena = arena;
                    rt_mutex_release(&mutex_arena);

                    if(auxArena.IsEmpty()){
                        WriteInQueue(&q_messageToMon,new MessagePosition()); // a new empty Position
                    }else{
                        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
                        img = camera.Grab();
                        rt_mutex_release(&mutex_camera);

                        position = img.SearchRobot(auxArena).front(); // return the first found robot in the list of foud robots
                        WriteInQueue(&q_messageToMon,new MessagePosition(MESSAGE_CAM_POSITION,position));

                        rt_task_set_periodic(NULL,TM_NOW,period);
                        rt_mutex_acquire(&mutex_actionType,TM_INFINITE);
                        actionType = CAMERA_STREAM;
                        rt_mutex_release(&mutex_actionType);                        
                    }

                    break;
                default :
                    break;
            }
        }
    }
     
}
// cette tache se charge de gérer les communication avec la camera

void Tasks::CamCommunicationTask(void* arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // on synchronize avec la barrier
    rt_sem_p(&sem_barrier,TM_INFINITE);
   
    while(1){

        int tempCommand;
        bool status;
        Message* m;
        // attente d'un signal d'une demande de communicationac le moniteur
        rt_sem_p(&sem_CamCommunication,TM_INFINITE); // prise du semaphore de communication
        // prise du mutex de la camera pour les commande 
        rt_mutex_acquire(&mutex_commandCamera,TM_INFINITE); 
        tempCommand = commandCamera;
        rt_mutex_release(&mutex_commandCamera);
        switch(tempCommand){
            case MESSAGE_CAM_OPEN: 
                cout << "Camera Opening ..." << endl << flush;
                // prise du mutex
                rt_mutex_acquire(&mutex_camera,TM_INFINITE);
                status = cammera.Open();
                cout << "status " << status << endl << flush;
                
                rt_mutex_release(&mutex_camera);
                if(status){
                    m = new Message(MESSAGE_ANSWER_ACK);
                    WriteInQueue(&q_messageToMon,m);
                    
                    rt_mutex_acquire(&mutex_continueStream, TM_INFINITE);
                    canStream = true ;
                    rt_mutex_release(&mutex_continueStream);
                    

                    rt_mutex_acquire(&mutex_actionType, TM_INFINITE);
                    actionCamera = CAMERA_STREAM ;
                    rt_mutex_release(&mutex_actionType);
                    

                    rt_mutex_acquire(&mutex_period, TM_INFINITE);
                    period = rt_timer_ns2ticks(100000000); // frequence d'echantillonage 100 ms 
                    rt_mutex_release(&mutex_period);
                    
                    
                    rt_sem_v(&sem_start_Stream);
                    

                }else{
                    cout << "something went wrong" << status << endl << flush;
                    m = new Message(MESSAGE_ANSWER_NACK);
                    WriteInQueue(&q_messageToMon,m);
                }
                break;
            case MESSAGE_CAM_CLOSE:
                cout << "Camera Closing ..." << endl << flush;
                m = new Message( MESSAGE_ANSWER_ACK);
                
                rt_mutex_acquire(&mutex_continueStream,TM_INFINITE);
                canStream = false;
                rt_mutex_release(&mutex_continueStream);
                // Camera properly closed
                WriteInQueue(&q_messageToMon,m);
                break;
            case MESSAGE_CAM_ASK_ARENA:
                cout << "Camera Asking arena..." << endl << flush;
                rt_mutex_acquire(&mutex_actionType,TM_INFINITE);
                actionCamera = CAMERA_ASK_ARENA;
                rt_mutex_release(&mutex_actionType);
                break;
            case MESSAGE_CAM_ARENA_CONFIRM:
                cout << "Camera Confirm Arena ..." << endl << flush; 
               // TODO
                rt_mutex_acquire(&mutex_arenaConfirmed,TM_INFINITE);
                arenaConfirmed = true;
                rt_mutex_release(&mutex_arenaConfirmed);

                rt_sem_v(&sem_arenaResult);
                break;
            case MESSAGE_CAM_ARENA_INFIRM:
                cout << "Camera Infirm Arena ..." << endl << flush;
                // TODO
                rt_mutex_acquire(&mutex_arenaConfirmed,TM_INFINITE);
                arenaConfirmed = false;
                rt_mutex_release(&mutex_arenaConfirmed);

                rt_sem_v(&sem_arenaResult);
                break;
            case MESSAGE_CAM_POSITION_COMPUTE_STOP:
                cout << "Stoping camera..." << endl << flush;
                rt_mutex_acquire(&mutex_actionType,TM_INFINITE);
                actionCamera = CAMERA_STREAM; // TODO ask what COMPUTE STOP MEANS ?
                rt_mutex_release(&mutex_actionType);
                break;
            case MESSAGE_CAM_POSITION_COMPUTE_START:
                cout << "Start Finding camera pos camera..." << endl << flush;
                rt_mutex_acquire(&mutex_actionType,TM_INFINITE);
                actionCamera = CAMERA_FIND_POSITION; 
                rt_mutex_release(&mutex_actionType);
                break;
            default :
                break;
        }

    }
}

void Tasks::RefreshWatchDog(void* arg){
    int rs_status;
    //Synchronization barrier
    //rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_refreshWatchDog, TM_INFINITE);
    
    //Beginning of the Task
    //rt_task_set_periodic(&th_refreshWD, TM_NOW, 955000000);
    rt_task_set_periodic(&th_refreshWatchDog, TM_NOW, rt_timer_ns2ticks(1000000000));
   
    
    while(1) {
        rt_task_wait_period(NULL);
        cout << "WatchDog Refresh" << endl;
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs_status = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs_status) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(robot.ReloadWD());
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}