function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      let values = this.responseText.split('|');
      if(values.length==1){
        document.getElementById('data_div').innerHTML = this.responseText;
      }else{
        for (let i=0; i<(values.length); i++){
          let value = values[i].split(';');
          if(value[0]=='display'){
            document.getElementById('lc').style.display = value[1];
          }else if(value[0]=='btn1'){
            if(value[1]=='0'){document.getElementById("btn1").disabled = true;}
            else{document.getElementById("btn1").disabled = false;}
          }else{
            document.getElementById(value[0]).innerHTML = value[1] + '';
          }
        }
      }
    }
  };
  xhttp.open('GET', '__WEB_SETTINGS_TIMER_HANDLER__', true);
  xhttp.timeout=1000;
  xhttp.send();
  var timer = window.setTimeout('getData()', __WEB_SETTINGS_TIMER_INTERVAL__);
}

getData();
