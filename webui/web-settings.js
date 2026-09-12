function btnClick(btn) {
  var xhttp = new XMLHttpRequest();
  xhttp.open('POST','?'+btn+'=',true);
  xhttp.timeout=5000;
  xhttp.send();
}

function urlencode(str) {
  return encodeURIComponent(String(str))
    .replace(/[!'()*]/g, c =>
      '%' + c.charCodeAt(0).toString(16).toUpperCase()
    )
    .replace(/%20/g, '+');
}

const collection = document.getElementsByClassName('t1');
for (let i = 0; i < collection.length; i++){
  var button = document.createElement('button');
  button.type = 'button';
  button.innerHTML = 'S';
  button.className = 'sb';
  button.addEventListener("click", function(event){
    let t1=this.parentNode.previousSibling.firstChild;
    if(t1.nextSibling!=null){
      if(t1.nextSibling.className==='toggle') t1=this.parentNode.previousSibling.childNodes[5].childNodes[1].childNodes[1];
    }

    if(t1.checkValidity()===false){
      alert('Fehler');
      return;
    }

    var name=t1.getAttribute('name');
    var val=0;
    if(t1.nodeName=='SELECT'){
      val=t1.options[t1.selectedIndex].value;
    }else if(t1.nodeName=='INPUT'){
      if(t1.type=='checkbox'){
        if(t1.checked){val=1;}
        else{val=0;}
      }else{
        val=t1.value;
        if(t1.className==='fl1') val*=10;
        else if(t1.className==='fl2') val*=100;
        else if(t1.className==='fl3') val*=1000;
      }
    }else if(t1.nodeName=='FIELDSET'){
      var ele = t1.getElementsByTagName('INPUT');
      name=ele[0].getAttribute('name');
      for(i=0; i<ele.length; i++) if(ele[i].checked) val|=(1<<i);
    }else{
      val=t1.value;
    }

    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function(){
      if (this.readyState==4 && this.status==200){
        alert(this.responseText);
      }
    };
    xhttp.open('GET','?SAVE=&'+name+'='+urlencode(val),true);
    xhttp.timeout=10000;
    xhttp.send();
  });
  collection[i].appendChild(button);
}

document.getElementById('lc').style.display = 'none';

function copyStringToClipboard (str) {
  var el = document.createElement('textarea');
  el.value = str;
  el.setAttribute('readonly', '');
  el.style = {position: 'absolute', left: '-9999px'};
  document.body.appendChild(el);
  el.select();
  document.execCommand("copy");
  document.body.removeChild(el);
}
