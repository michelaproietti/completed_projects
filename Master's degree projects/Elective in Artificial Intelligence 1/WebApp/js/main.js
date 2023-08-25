sessionStorage.setItem('tutorial_visibility', 'false');
sessionStorage.setItem('difficulty', 0);

$(document).ready(function() {

    $("#worldDay").prop("checked", true);

    $("#button_tutorial-description").hide();

    $("#button_tutorial").click(function() {
        if (sessionStorage.getItem('tutorial_visibility') == 'false') {
            $("#button_tutorial-description").fadeIn(1000);
            sessionStorage.setItem('tutorial_visibility', 'true');
        } else {
            $("#button_tutorial-description").fadeOut(1000);
            sessionStorage.setItem('tutorial_visibility', 'false');
        }
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

    $("#buttonIP").click(function(){
        sessionStorage.setItem('ip_pepper', $("#IP").val());
        $("#IP").attr("placeholder", sessionStorage.getItem("ip_pepper"));
        $('#IP').val('')
    });

    // I verify that a previously submitted ip exists and if it does not exist I use the default one
    if (sessionStorage.getItem("ip_pepper") === null) {
        sessionStorage.setItem('ip_pepper', '172.16.187.128');
    }
    $("#IP").attr("placeholder", sessionStorage.getItem("ip_pepper"));

});
