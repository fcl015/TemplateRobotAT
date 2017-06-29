#include "at_modelo.h"

#define PIN_FRENO 12
#define PIN_LLAVE 1
#define PIN_PARO 0
#define TIEMPO_LECTURA_MS 1000
#define SEGURO_ON 5
#define SEGURO_OFF 6

//Respuestas del servidor al monitor de alarma
#define PARO_FISICO 3
#define PARO_FISICO_LIBERADO 4
#define NUM_MAX_REINTENTOS_INICIO 3
//respuestas de la reanudacion de monitor alarma
#define REINTENTAR 1
#define SALIR -1
#define COMANDO_COMPLETADO 255

ATModelo::ATModelo(QObject *parent) : QObject(parent),
    stopReceived(false)
{
    salida = false;
    estadoParoInicio = false;
    socketPrincipalConectado = false;
    posicionSegura.x = 0;
    posicionSegura.y = 0;
    posicionSegura.z = 770;
    //posicionSegura.z = 657;//itesm
}

void ATModelo::Salida(bool opcion){
    if(opcion)
        aPosicionEspera(robot);

    mutexSalida.lock();
    salida = true;
    mutexSalida.unlock();

    robot->salidasActuadorPLC(PIN_FRENO,false);
    ATUtils::utils_espera_process_events(500);
    robot->sync(false);
    robot->switchON(false);
    robot->plcReady(false);
    robot->desconectar();

    while(!robotParoFinalizado)
        ATUtils::utils_espera_process_events(100);
    delete robot;
    emit finHilo();
}

void ATModelo::controladorSetStop(bool estado){
    QMutex mutexControladorParo;
    mutexControladorParo.lock();
    stopReceived = estado;
    mutexControladorParo.unlock();
}

void ATModelo::inicializarRobot(){
    int repeticiones = 0;
    int valor        = 0;

    robot->setPinParo(0);
    robot->switchON(true);
    ATUtils::utils_espera_process_events(500);
    robot->salidasActuadorPLC(PIN_FRENO,true);

    secuenciaInicial:

    // Plc ready ON
    valor = robot->plcReady(true);
    switch (comprobarComando(robot,valor,&repeticiones,PUERTO_CALIBRACION)) {
    case SALIR:
        errorDeConexion(PUERTO_CALIBRACION);
        return;
    case REINTENTAR:
        goto secuenciaInicial;
    }

    // Mov sin cinematica
    valor = robot->movSinCinematica();
    switch (comprobarComando(robot,valor,&repeticiones,PUERTO_CALIBRACION)) {
    case SALIR:
        errorDeConexion(PUERTO_CALIBRACION);
        return;
    case REINTENTAR:
        goto secuenciaInicial;
    }

    // HOMMING
    valor = robot->homming();
    switch (comprobarComando(robot,valor,&repeticiones,PUERTO_CALIBRACION)) {
    case SALIR:
        errorComunicacion(PUERTO_CALIBRACION);
        return;
    case REINTENTAR:
        goto secuenciaInicial;
    }

    // Sync ON
    if(repeticiones < NUM_MAX_REINTENTOS_INICIO){
        valor = robot->sync(SYNC_ON);
        switch (comprobarComando(robot,valor,&repeticiones,PUERTO_CALIBRACION)) {
        case SALIR:
            errorComunicacion(PUERTO_CALIBRACION);
            return;
        }
    }

    if(comprobarErrores(robot,&repeticiones))
        goto secuenciaInicial;

    if(repeticiones == NUM_MAX_REINTENTOS_INICIO){
        errorComunicacion(PUERTO_CALIBRACION);
        return;
    }

}

void ATModelo::errorComunicacion(int numSocket){
    qDebug()<<"Error de conexion";
}

bool ATModelo::getStopReceived(){
    bool aux;
    mutexSenalParo.lock();
    aux = stopReceived;
    mutexSenalParo.unlock();
    return aux;
}

void ATModelo::Ejecucion(){

    Punto_3D punto_inicial1;
    Punto_3D punto_final1;

    int numCasilla=1;
    double offsetX=22.5;
    double offsetY=38.97;
    double offsetXaux=0;
    double offsetYaux=0;
    double offsetXauxInicio=0;
    double offsetXauxFinal=0;

    punto_inicial1.x=45.0;
    punto_inicial1.y=162.0;
    punto_inicial1.z=955.0;

    punto_final1.x=49.0;
    punto_final1.y=171.0;
    punto_final1.z=957.0;


    qDebug()<<"Iniciando Ejecucion";
    robotEjecucion = new ATRobot(PUERTO_EJECUCION);
    banderaEstadoEjec = true;

    //Ir a la posicion de espera
    aPosicionEspera(robotEjecucion);

//    do{
//        if(robotEjecucion->setPunto(MOV_LINEAL,200,0,posicionSegura.z,300) != COMANDO_EJECUTADO){

//            break;
//        }
//        if(robotEjecucion->setPunto(MOV_LINEAL,-200,0,posicionSegura.z,300) != COMANDO_EJECUTADO){
//            break;
//        }
//        if(getStopReceived()){
//            break;
//        }
//    }while(banderaEstadoEjec);

    for(int contadorFilas=0;contadorFilas<3;contadorFilas++){
        for(int contadorColumnas=0;contadorColumnas<numCasilla;contadorColumnas++){

        }
    }

    detenerEjecucion();

}

void ATModelo::setStopReceived(bool estado){
    if(estado){
        robot->softStop();
        ATUtils::utils_espera<std::chrono::seconds>(2);
        //robot->getPosActual(&xReanudar,&yReanudar,&zReanudar);
    }
    mutexSenalParo.lock();
    stopReceived = estado;
    mutexSenalParo.unlock();
}

void ATModelo::conectarRobot(){
    robot = new ATRobot(PUERTO_CALIBRACION);
    if(!robot->isServerConnected()){
        emit errorDeConexion(PUERTO_CALIBRACION);
        return;
    }

    robot->limpiarErrores();

    if(!robot->statusPinEntrada(PIN_PARO)){
        estadoParoInicio = true;
        emit errorParo();
    }
    inicializarRobot();
    //bloquear con mutex
    socketPrincipalConectado = true;
}

void ATModelo::lanzarAlarmaMonitor(){
    QFuture<void>future;
    future = QtConcurrent::run(this,&ATModelo::alarmaMonitor);
}

void ATModelo::detenerEjecucion(){
    reanudar = true;
    //    banderaEstadoEjec = false;
    mutexSenalParo.lock();
    stopReceived = false;
    mutexSenalParo.unlock();
    emit errorMovimiento();
    robotEjecucion->desconectar();
    delete robotEjecucion;
}

/*Metodos privados*/
bool ATModelo::comprobarErrores(ATRobot *robotAux, int * repeticiones){
    if(robotAux->comprobarErrores(WARNING) && *repeticiones < NUM_MAX_REINTENTOS_INICIO){
        robotAux->limpiarErrores();
        *repeticiones += 1;
        return true;
    }
    if(robotAux->comprobarErrores(CRITICAL) && *repeticiones < NUM_MAX_REINTENTOS_INICIO){
        robotAux->limpiarErrores();
        *repeticiones += 1;
        return true;
    }
    return false;
}

int ATModelo::comprobarComando(ATRobot *robotAux, int valor, int *repeticiones, int puertoSocket){
    if(valor == ERROR_COMUNICACION){
        errorDeConexion(puertoSocket);
        return SALIR;
    }
    else if(valor == ERROR_COMANDO_NO_EJECUTADO){
        *repeticiones += 1;
        robotAux->limpiarErrores();
        return REINTENTAR;
    }
    return COMANDO_COMPLETADO;
}


void ATModelo::alarmaMonitor(){
    QThread::currentThread()->setPriority(QThread::TimeCriticalPriority);

    do{
        ATUtils::utils_espera_process_events(100);
    }while(!socketPrincipalConectado);

    robotParo           = new ATRobot(PUERTO_PARO);
    robotParoFinalizado = false;
    salida              = false;

    u_int16_t read; //buffer donde se guarda la respuesta del PLC
    u_int16_t maxread       = -1;
    bool errorReanudacion   = false;
    int numIntentosError    = 0, valor = 0;

    if(estadoParoInicio){
        robotParo->desconectar();
        robotParoFinalizado = true;
        delete robotParo;
        return;
    }

    do{
        read = maxread;
        read = robotParo->leer(TIEMPO_LECTURA_MS);

        if(read == PARO_FISICO){
            emit enParo(true);
            QCoreApplication::processEvents(QEventLoop::AllEvents,100);

            mutexSenalParo.lock();
            stopReceived = true;
            mutexSenalParo.unlock();

            robotParo->limpiarErrores();

secuenciaReanudacion:

            if(numIntentosError >= NUM_MAX_REINTENTOS_INICIO){
                emit enParo(false);
                stopReceived = false;
                emit errorDeConexion(PUERTO_PARO);
                continue;
            }

            if(!errorReanudacion){
                valor = robotParo->plcReady(true);
                switch (comprobarComando(robotParo,valor,&numIntentosError,PUERTO_PARO)) {
                case SALIR:
                    errorDeConexion(PUERTO_PARO);
                    return;
                case REINTENTAR:
                    errorReanudacion = false;
                    goto secuenciaReanudacion;
                }
                valor = robotParo->homming(true);
                switch (comprobarComando(robotParo,valor,&numIntentosError,PUERTO_PARO)) {
                case SALIR:
                    errorDeConexion(PUERTO_PARO);
                    return;
                case REINTENTAR:
                    errorReanudacion = false;
                    goto secuenciaReanudacion;
                }
            }
            errorReanudacion = false;

            if(comprobarErrores(robotParo,&numIntentosError))
                goto secuenciaReanudacion;

            valor = robotParo->sync(SYNC_FORZADO);
            switch (comprobarComando(robotParo,valor,&numIntentosError,PUERTO_PARO)) {
            case SALIR:
                if(!reconectar(PUERTO_PARO)){
                    emit enParo(false);
                    stopReceived = false;
                    emit errorDeConexion(PUERTO_PARO);
                }
                robotParo->sync(SYNC_OFF);
                goto secuenciaReanudacion;
                return;
            case REINTENTAR:
                errorReanudacion = false;
                goto secuenciaReanudacion;
            }

            if(comprobarErrores(robotParo,&numIntentosError))
                goto secuenciaReanudacion;


            u_int16_t respuestaPostParo = -1;

            //Espera a que se libere el paro
            do{
                respuestaPostParo = -1;
                ATUtils::utils_espera_process_events(100,QEventLoop::ExcludeUserInputEvents);
                respuestaPostParo = robotParo->leer();

            }while(respuestaPostParo != PARO_FISICO_LIBERADO);

            stopReceived = false;
            emit enParo(false);

        }
    }while(!salida);
    robotParo->desconectar();
    delete robotParo;

    robotParoFinalizado = true;
}


void ATModelo::aPosicionEspera(ATRobot *robotE){
    double x,y,z;
    robotE->getPosActual(&x,&y,&z);
    bool inHome = enHome(x,y,z);

    if(!inHome){
        robotE->setPunto(MOV_LINEAL,x,y,posicionSegura.z,300);
        robotE->setPunto(MOV_LINEAL,posicionSegura.x,posicionSegura.y,
                        posicionSegura.z,300);
    }

    else{
        robotE->setPunto(MOV_LINEAL,posicionSegura.x,posicionSegura.y,
                        posicionSegura.z,300);
    }
}

bool ATModelo::reconectar(int numSocket){
    if(numSocket == 1){
        robot->desconectar();
        delete robot;
        robot = new ATRobot(8091);

        robot->limpiarErrores();
        ATUtils::utils_espera_process_events(200);
        if(robot->getEstadoSocket() == UnconnectedState)
            return false;
    }
    else if(numSocket == 3){
        robotParo->desconectar();
        delete robotParo;
        robotParo = new ATRobot(8093);

        robotParo->limpiarErrores();
        ATUtils::utils_espera_process_events(200);

        if(robotParo->getEstadoSocket() == UnconnectedState)
            return false;
    }
    return true;
}

bool ATModelo::enHome(double x,double y,double z){
    double tolerancia = 10;

    return (sqrt( pow(x - posicionSegura.x,2) +
                  pow(y - posicionSegura.y,2) +
                  pow(z - posicionSegura.z,2) )
            <= tolerancia) ? true : false;
}
