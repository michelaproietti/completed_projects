import * as GAME from './game.js'

// log display function
function append(text) {
    console.log(text);
} 

// websocket global variable
var websocket = null;

export var connessioneStabilita = -1;

export function wsrobot_connected() {
    var connected = false;
    if (websocket!=null)
        console.log("websocket.readyState: "+websocket.readyState)
    if (websocket!=null && websocket.readyState==1) {
        connected = true;
    }
    console.log("connected: "+connected)
    return connected;
}

export function wsrobot_init(port) {
    var ip = sessionStorage.getItem("ip_pepper") //"172.16.187.128" "127.0.0.1" "127.0.1.1"
    var url = "ws://"+ip+":"+port+"/websocketserver";
    console.log(url);
    websocket = new WebSocket(url);

    websocket.onmessage = function(event){
        append("message received: "+event.data);
        if(event.data=="victoryRating"){
            location.href='./ratings.html'
        }else{
            GAME.moveForPlanner(event.data);
        }
    } 

    websocket.onopen = function(){
        connessioneStabilita=1;
        append("connection received");
    } 

    websocket.onclose = function(){
        append("connection closed");

    }

    websocket.onerror = function(){
        window.alert("Connection problems! Returning automatically to the main menu")
        location.href='./main.html'
        append("!!!connection error!!!");
    }

}

export function wsrobot_quit() {
    websocket.close();
    websocket = null;
}

export function wsrobot_send(data) {
if (websocket!=null)
    websocket.send(data);
}


