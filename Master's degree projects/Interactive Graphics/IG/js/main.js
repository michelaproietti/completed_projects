sessionStorage.setItem('passaggioScore', 'score: 0');
sessionStorage.setItem('dayOrNight', 'false');

$(document).ready(function() {

    $("#worldDay").prop("checked", true);

    $("#easy-description").hide();
    $("#medium-description").hide();
    $("#hard-description").hide();
    $("#button_tutorial-description").hide();

    $("#easy").mouseover(function() {
      $("#easy-description").fadeIn(1000);
    });
    $("#easy").mouseout(function() {
      $("#easy-description").hide();
    });

    $("#medium").mouseover(function() {
      $("#medium-description").fadeIn(1000);
    });
    $("#medium").mouseout(function() {
      $("#medium-description").hide();
    });

    $("#hard").mouseover(function() {
      $("#hard-description").fadeIn(1000);
    });
    $("#hard").mouseout(function() {
      $("#hard-description").hide();
    });

    $("#button_tutorial").mouseover(function() {
      $("#button_tutorial-description").fadeIn(1000);
    });
    $("#button_tutorial").mouseout(function() {
      $("#button_tutorial-description").hide();
    });

    $("#worldDay").click(function(){
        sessionStorage.setItem('dayOrNight', 'false');
        $("div.title-hour-select").css("color","black");
        $("html").css("background-image","url(assets/textures/sunny_sky.jpg)");
        $("body").css("background-image","url(assets/textures/sunny_sky.jpg)");
    });

    $("#worldSunset").click(function(){
        sessionStorage.setItem('dayOrNight', 'true');
        $("div.title-hour-select").css("color","white");
        $("html").css("background-image","url(assets/textures/night_sky.jpg)");
        $("body").css("background-image","url(assets/textures/night_sky.jpg)");
    });

    $("#easy").click(function(){
        sessionStorage.setItem('levelPass', 'easy');
        location.href='./game.html'
    });

    $("#medium").click(function(){
        sessionStorage.setItem('levelPass', 'medium');
        location.href='./game.html'
    });

    $("#hard").click(function(){
        sessionStorage.setItem('levelPass', 'hard');
        location.href='./game.html'
    });

});
