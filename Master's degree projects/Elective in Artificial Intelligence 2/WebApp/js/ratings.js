sessionStorage.setItem('difficulty', 0);

$(document).ready(function() {

    $("#round-1").click(function(){
        sessionStorage.setItem('difficulty', 1);
    });

    $("#round-2").click(function(){
        sessionStorage.setItem('difficulty', 2);
    });
    
    $("#round-3").click(function(){
        sessionStorage.setItem('difficulty', 3);
    });

    $("#round-4").click(function(){
        sessionStorage.setItem('difficulty', 4);
    });

    $("#round-5").click(function(){
        sessionStorage.setItem('difficulty', 5);
    });
    
    $("#restart").click(function(){
        location.href='./game.html'
    });

});