import * as CLIENT from './clientQuiz.js'

sessionStorage.setItem('tutorial_visibility', 'false');

$(document).ready(function() {

    CLIENT.wsrobot_init(9020)


    $("#worldDay").prop("checked", true);
    $("#start_game").click(function(){
        CLIENT.wsrobot_send("goDirectlyGame_")
        goToMain()
    });

    setTimeout(function (){
        CLIENT.wsrobot_send("doQuiz_")      
    }, 500);

});

export function goToMain(){
    location.href='./main.html'
}
