#include <Python.h>
#include <chrono>
#include <string>
#include <iostream>
#include <thread>

void log(std::string strr)
{
	std::cout<<strr<<std::endl;
}

PyObject* initMidiSegPlay(const char* fname) {

    log("initMidiSegPlay~");
	PyObject* pModule;
	PyObject* pFunc;
	//call python function, with multiple parameters
	PyObject* pRet = NULL;
    log("PyImport_ImportModule~");
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./')");
	pModule = PyImport_ImportModule("midiseg_cc");
	log("PyImport_ImportModule OK");
	if (NULL == pModule)
	{
		log("PyImport_ImportModule NULL~");
		return pRet;
	}
	log("PyObject_GetAttrString~");
	pFunc = PyObject_GetAttrString(pModule, "initMidiSegPlay");
	if (pFunc)
	{
		PyObject* pParams = NULL;
        pParams = PyTuple_New(1);		  //create tuple to put parameters
		log("PyTuple_SetItem~");
		PyTuple_SetItem(pParams, 0, PyUnicode_FromString(fname));
		log("PyEval_CallObject~");
		pRet = PyEval_CallObject(pFunc, pParams);
	}
	return pRet;
}

void playNext(PyObject* msp, std::chrono::time_point<std::chrono::steady_clock> starttime) {   
	PyObject* pModule;
	PyObject* pFunc;
	
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('./')");
	pModule = PyImport_ImportModule("midiseg_cc");
	if (NULL == pModule)
	{
		log("PyImport_ImportModule NULL~");
		exit(0);
        return;
	}
	pFunc = PyObject_GetAttrString(pModule, "playNext");

	if (pFunc)
	{
		PyObject* pParams = NULL;
		pParams = PyTuple_New(2);		  //create tuple to put parameters
		PyTuple_SetItem(pParams, 0, msp);
        
        auto current = std::chrono::steady_clock::now();
        std::chrono::duration<double> acctime = current-starttime;
        PyTuple_SetItem(pParams, 1, PyFloat_FromDouble(acctime.count()));
		
		PyEval_CallObject(pFunc, pParams);
	}
	
}

int main()
{
	Py_Initialize();
	
	log("in main~");
    auto pyobj = initMidiSegPlay("kmhm.mid");
	log("initMidiSegPlay ok");
	auto start = std::chrono::steady_clock::now();
    while(1)
    {
		
        playNext(pyobj, start);
		
    }

    return 0;
}