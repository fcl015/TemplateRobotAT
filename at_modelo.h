#ifndef ATMODELO_H
#define ATMODELO_H

#include <QObject>
#include <QMutex>
#include <QDebug>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>
#include <AutomatischeM/at_robot.h>

#include "estado_programa.h"

class ATModelo : public QObject
{
    Q_OBJECT
public:
    bool stopReceived; ///< variable booleana que se usa para saber el estado de "stop"
    explicit ATModelo(QObject *parent = 0);
    /**
     * @brief Salida
     * regresa el robot a home si es true, posteriormente ejecuta emite una señal
     * para iniciar el proceso de cierre de modelo
     */
    void Salida(bool opcion);
    void controladorSetStop(bool estado);
    bool getStopReceived();
//    bool getMovTerminado();
    void Ejecucion();
    void setStopReceived(bool opcion);
    void conectarRobot();
    /**
     * @brief lanzarAlarmaMonitor
     * Metodos que crea el hilo que monitorea el paro de emergencia y la llave
     */
    void lanzarAlarmaMonitor();
    void resetNumPosObjetivo();
    void aPosicionEspera(ATRobot* robotE);
    void inicializarRobot();
    bool reconectar(int numSocket);

signals:
    /**
     * @brief finHilo
     * se emite cuando finaliza la tarea del hilo de ejecucion
     */
    void finHilo();
    void errorDeConexion(int puerto);
    void errorMovimiento();
    void errorParo();
    void enParo(bool opcion);
public slots:

private:
    ATRobot*             robot;             /*Robot encargado de hacer movimiento de calibracion*/
    ATRobot*             robotEjecucion;    /*Robot encargado ejecutar las trayectorias de movimiento*/
    ATRobot*             robotParo;         /*Instancia encargada de monitorear la senal de paro y la reanudacion*/

    QMutex mutexSalida;
    QMutex mutexSenalParo;
    Punto_3D posicionSegura;
    bool salida;
    bool reanudar;
    bool robotParoFinalizado;
    bool socketPrincipalConectado;
    bool estadoParoInicio;
    bool banderaEstadoEjec;
    void alarmaMonitor();
    void detenerEjecucion();
    void errorComunicacion(int numSocket);
    bool enHome(double x, double y, double z);
    bool comprobarErrores(ATRobot * robotAux, int *repeticiones);
    int comprobarComando(ATRobot * robotAux, int valor, int *repeticiones, int puertoSocket);
};

#endif // ATMODELO_H
