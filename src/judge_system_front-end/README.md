# judgesystem

> A JudgeSystem based on Vue.js project.

## Build Setup

``` bash
# install dependencies
npm install

# serve with hot reload at localhost:YOUR_PORT
npm run dev

# build for production with minification
npm run build

# build for production and view the bundle analyzer report
npm run build --report
```

For a detailed explanation on how things work, check out the [guide](http://vuejs-templates.github.io/webpack/) and [docs for vue-loader](http://vuejs.github.io/vue-loader).
## Quick Start
需要在Ubuntu中安装POCO以作C++后端。
### How to install POCO
1. 官网下载POCO并解压
```bash
tar -xzf poco-1.12.4-all.tar.gz
```
2. 进入目录
```bash
cd poco-1.12.4-all/
```
3. 安装POCO依赖库(可略过)
```bash
ODBC数据库
sudo apt-get install unixodbc
MySQL
sudo apt-get install mysql
```
4. 如果未安装ODBC及MySQL需要在configue中加入语句忽略
```bash
./configure --shared --prefix=/usr/local/poco --omit=Data/ODBC,Data/SQLite  --everything
```
5. make
```
make && make install
```
### Example
C++后端例程，放在指定位置：
```poco_server.h```
```c++
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/HTTPServer.h>
#include <Poco/Net/HTTPRequestHandler.h>
#include <Poco/Net/HTTPRequestHandlerFactory.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/Net/HTTPServerRequest.h>
#include <Poco/Net/HTTPServerResponse.h>
#include <Poco/Util/ServerApplication.h>
 
#include <iostream>
#include <string>
#include <vector>
#include <Poco/JSON/Parser.h>
#include <Poco/StreamCopier.h>
#include <Poco/Util/ServerApplication.h>
#include <Poco/JSON/Parser.h>
#include <Poco/StreamCopier.h>

#include "const.h"
using namespace Poco::JSON;
using namespace std;
using namespace Poco::Net;
using namespace Poco::Util;
vector<int> values;
class MyRequestHandler : public HTTPRequestHandler
{
public:
	virtual void handleRequest(HTTPServerRequest &req, HTTPServerResponse &resp)
	{
		Application& app = Application::instance();
		app.logger().information("Request from " + req.clientAddress().toString());
		
		std::string Json_str;
        Poco::StreamCopier::copyToString(req.stream(), Json_str);
        std::cout<<Json_str<<std::endl;

        Poco::JSON::Parser parser;
        Poco::Dynamic::Var json = parser.parse(Json_str);
        Poco::JSON::Object::Ptr object_ptr = json.extract<Poco::JSON::Object::Ptr>();
        isModifing = true;
        if(values.empty()){
            for(int i = 0; i < keys.size(); ++i){
                int t = object_ptr->getValue<int>(keys[i]);
                values.push_back(t);
            }
        }else {
            for(int i = 0; i < keys.size(); ++i){
                int t = object_ptr->getValue<int>(keys[i]);
                values[i] = t;
            }
        }
        isModifing = false;
		//解决跨域问题
		resp.add("Access-Control-Allow-Origin","*");
    resp.add("Access-Control-Allow-Methods","POST,GET,OPTIONS,DELETE");
    resp.add("Access-Control-Max-Age","3600");
    resp.add("Access-Control-Allow-Headers","x-requested-with,content-type");
    resp.add("Access-Control-Allow-Credentials","true");

		resp.setStatus(HTTPResponse::HTTP_OK);
		resp.setContentType("text/json");

		Poco::JSON::Object bodyObj;
    bodyObj.set("status", 0);
    std::ostringstream ss;
    bodyObj.stringify(ss);
    std::string body;
    body = ss.str();
    resp.setStatus(HTTPResponse::HTTP_OK);
    ostream &out = resp.send();
    out << body;
		std::cout<<body<<std::endl;
	}
};
 
class MyRequestHandlerFactory : public HTTPRequestHandlerFactory
{
public:
	virtual HTTPRequestHandler* createRequestHandler(const HTTPServerRequest &)
	{
		return new MyRequestHandler;
	}
};
 
class MyServerApp :public ServerApplication
{
protected:
	int main(const vector<string> &)
	{
		HTTPServer s(new MyRequestHandlerFactory, ServerSocket(1111), new HTTPServerParams); // 1111是端口
 
		s.start();
		cout << endl << "Server started" << endl;
 
		waitForTerminationRequest();  // wait for CTRL-C or kill
 
		cout << endl << "Shutting down..." << endl;
 
		s.stop();
 
		return Application::EXIT_OK;
	}
};
```
```const.h```
```c++
#include <string>
#include <vector>
bool isModifing = false;
std::vector<std::string> keys = {
    "remaining_time",
    "blue_base_blood",
    "red_base_blood",
    "r1_blood",
    "b1_blood",
    "r2_blood",
    "b2_blood",
    "r3_blood",
    "b3_blood",
    "r4_blood",
    "b4_blood",
    "r5_blood",
    "b5_blood",
    "r6_blood",
    "b6_blood",
    "r7_blood",
    "b7_blood",
    "r_outpost",
    "b_outpost",
    "energy",
    "r_darts",
    "b_darts",
    "bullet_dose"
};
```
```poco_server.cpp```
需要多线程操作，并对全局变量```values```互斥访问，在此仅简单设计标记符```isModifying```区分```values```是否正在修改。
```c++
#include "poco_server.h"
#include <thread>
int server_thread(int argc, char **argv) {
	MyServerApp app;
	return app.run(argc, argv);
}
void start(){
	while (true)
	{
		sleep(1);
		// 简易互斥锁
		if(!isModifing) {
			for (int i=0;i<values.size();++i){
				cout<<keys[i]<<" "<<values[i]<<endl;
			}
		}
	}
}
int main(int argc, char **argv)
{

	thread thread_server(server_thread, argc, argv);
	thread thread_2(start);
	thread_2.join();
	thread_server.join();
	return 0;
}
```
