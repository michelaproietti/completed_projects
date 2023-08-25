sessionStorage.setItem('tutorial_visibility', 'false');

$(document).ready(function() {

    $("#worldDay").prop("checked", true);

    $("#start_quiz").click(function(){
        location.href='./quiz.html'
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
