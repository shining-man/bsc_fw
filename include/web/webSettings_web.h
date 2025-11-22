// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "defines.h"

/* Minifier
 * CSS:  https://www.toptal.com/developers/cssminifier
 * JS:   https://www.giftofspeed.com/javascript-compressor/
 * GZIP: https://facia.dev/tools/compress-decompress/gzip-compress/
 */

 #ifndef HTML_MINIFY
 const char webSettingsStyle[] PROGMEM = R"rawliteral(
 <style>
 body {
   font-family: Arial, sans-serif;
   background-color: #f4f4f4;
   color: #333;
   margin: 0;
   padding: 0;
 }
 
 .topnav {
   display: flex;
   flex-wrap: nowrap;
   justify-content : center;
   align-items: center;
   gap: 10px;
   z-index: 10;
   position: sticky;
   top: 0;
   background: linear-gradient(90deg, #007bff, #0056b3);
   color: #fff;
   padding: 10px;
   text-align: center;
   box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
 }
 
 
 .topnav span {
   font-size: 1.5rem;
   font-weight: bold;
 }
 
 .hl {
   /*flex-grow: 1;*/
   padding: 10px;
 }
 
 .content {
   overflow-x: auto;
   max-width: 500px;
   margin: 20px auto;
   background: #fff;
   padding: 20px;
   border-radius: 10px;
   box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
 }
 
 table {
   width: 100%;
   max-width: 100%;
   border-collapse: collapse;
 }
 
 td, th {
   padding: 5px;
   border-bottom: none;
   white-space: normal;
   word-wrap: break-word;
   text-align: left;
 }
 
 .t1 {
   padding-left: 15px;
 }
 
 .Ctd {
 /*white-space: nowrap;*/
 white-space: normal !important;
 }
 
 summary {
 padding-bottom:10px;
 }
 
 .tr.sep {
 margin: 20px;
 }
 
 .sep {
   position: relative;
   z-index: 2;
   background-color: #007bff;
   color: white;
   font-weight: bold;
   padding: 0px;
   border-radius: 5px;
   text-align: center;
 }
 
 .sep:first-child {
   border-top: none;
 }
 
 .sep a {
   color: inherit;
 }
 
 button, .btnBack {
   border: none;
   padding: 6px 10px;
   border-radius: 5px;
   cursor: pointer;
   transition: background 0.3s;
   margin-left: 40px;
 }
 
 .btnBack {
   background-color: #007bff;
   color: white;
   margin-left: 0px;
 }
 
 .btn1, .btn2 {
   margin-left: 0px;
 }
 
 button:hover, .btnBack:hover {
   background-color: #0056b3;
 }
 
 input, select {
   /*width: 100%;*/
   padding: 5px;
   margin: 5px 0;
   border: 1px solid #ccc;
   border-radius: 5px;
   box-shadow: inset 0 1px 3px rgba(0, 0, 0, 0.1);
 }
 
 input:focus, select:focus {
   border-color: #007bff;
   outline: none;
 }
 
 input[type="checkbox"], input[type="radio"] {
   width: auto;
   margin-right: 5px;
 }
 
 .unit {
   display: inline;
   margin-left: 5px;
   font-weight: normal;
 }
 
 .help {
   position: relative;
   z-index: 0;
   font-size: 0.9rem;
   color: #666;
   padding: 6px;
   border-left: 3px solid #007bff;
   background: #f9f9f9;
   border-radius: 5px;
   margin-top: -8px;
 }
 
 hr {
   border-top: 1px dashed blue !important;
   height: 2px !important;
   margin: 0px 0 !important;
 }
 
 input:invalid {border:1px solid red;}
 input:valid {border:1px solid green;}
 
 
 .loading-container {
   position: relative;
   z-index: 3;
   margin: 0px;
   padding: 0px;
   min-height: 100vh;
   display: flex;
   align-items: center;
   justify-content: center;
   position: fixed;
   top: 50%;
   left: 50%;
   transform: translateX(-50%) translateY(-50%);
 }
 .loading-spinner {
   position: relative;
   z-index: 3;
   display: block;
   width: 100px;
   height: 100px;
   border: 7px solid #87ceeb3b;
   border-radius: 50%;
   border-top-color: #4682b4;
   animation: spin 1s linear infinite;
 }
 @keyframes spin {
   to {
   -webkit-transform: rotate(360deg);
   }
 }
 
 
 fieldset{padding:0px; margin:0px; border:0px;}
 
 .toggle {display: none;}
 .lbl-toggle {
   display: block;
   /*font-weight: bold;*/
   /*font-family: monospace;*/
   font-size: 1.0rem;
   /*text-transform: uppercase;*/
   text-align: left;
   padding: 2px 0 2px 10px;
   /*color: #A77B0E;*/
   background: #eaeaea;
   cursor: pointer;
   border-radius: 5px;
   transition: all 0.25s ease-out;
 }
 .lbl-toggle:hover {color: #464646;}
 .lbl-toggle::before {
   content: ' ';
   display: inline-block;
   border-top: 5px solid transparent;
   border-bottom: 5px solid transparent;
   border-left: 5px solid currentColor;
   vertical-align: middle;
   margin-right: .5rem;
   transform: translateY(-2px);
   transition: transform .2s ease-out;
 }
 .toggle:checked + .lbl-toggle::before {
   transform: rotate(90deg) translateX(-3px);
 }
 .collapsible-content {
   max-height: 0px;
   overflow: hidden;
   transition: max-height .25s ease-in-out;
 }
 .toggle:checked + .lbl-toggle + .collapsible-content {
   max-height: 100vh;
 }
 .toggle:checked + .lbl-toggle {
   border-bottom-right-radius: 0;
   border-bottom-left-radius: 0;
 }
 .collapsible-content .content-inner {
   background: #f3f3f3;
   border-bottom: 1px solid #eaeaea;
   border-bottom-left-radius: 7px;
   border-bottom-right-radius: 7px;
   padding: .5rem 1rem;
 }
 .profile-container{display: flex; gap: 5px; align-items: center; width: calc(100% + 2px);}
 .profile-container input{width: calc(50% - 2.5px); margin: 0;}
 .profile-container .profile{font-weight: bold; color: #666;}
 </style>
 )rawliteral";
 #else
 const char webSettingsStyle[] PROGMEM = R"rawliteral(
 <style>
 .hl,.topnav{padding:10px}.sep,.topnav{text-align:center}body{font-family:Arial,sans-serif;background-color:#f4f4f4;color:#333;margin:0;padding:0}.topnav{display:flex;flex-wrap:nowrap;justify-content:center;align-items:center;gap:10px;z-index:10;position:sticky;top:0;background:linear-gradient(90deg,#007bff,#0056b3);color:#fff;box-shadow:0 4px 6px rgba(0,0,0,.1)}.help,.sep{position:relative}.topnav span{font-size:1.5rem;font-weight:700}.content{overflow-x:auto;max-width:500px;margin:20px auto;background:#fff;padding:20px;border-radius:10px;box-shadow:0 4px 8px rgba(0,0,0,.1)}.btnBack,.sep{background-color:#007bff;color:#fff}table{width:100%;max-width:100%;border-collapse:collapse}td,th{padding:5px;border-bottom:none;white-space:normal;word-wrap:break-word;text-align:left}.t1{padding-left:15px}.Ctd{white-space:normal!important}summary{padding-bottom:10px}.tr.sep{margin:20px}.sep{z-index:2;font-weight:700;padding:0;border-radius:5px}.sep:first-child{border-top:none}.sep a{color:inherit}.btnBack,button{border:none;padding:6px 10px;border-radius:5px;cursor:pointer;transition:background .3s;margin-left:40px}.btn1,.btn2,.btnBack{margin-left:0}.btnBack:hover,button:hover{background-color:#0056b3}input,select{/*width:100%;*/padding:5px;margin:5px 0;border:1px solid #ccc;border-radius:5px;box-shadow:inset 0 1px 3px rgba(0,0,0,.1)}input:focus,select:focus{border-color:#007bff;outline:0}input[type=checkbox],input[type=radio]{width:auto;margin-right:5px}.unit{display:inline;margin-left:5px;font-weight:400}.help{z-index:0;font-size:.9rem;color:#666;padding:6px;border-left:3px solid #007bff;background:#f9f9f9;border-radius:5px;margin-top:-8px}hr{border-top:1px dashed #00f!important;height:2px!important;margin:0!important}input:invalid{border:1px solid red}input:valid{border:1px solid green}.loading-container{z-index:3;margin:0;padding:0;min-height:100vh;display:flex;align-items:center;justify-content:center;position:fixed;top:50%;left:50%;transform:translateX(-50%) translateY(-50%)}.loading-spinner{position:relative;z-index:3;display:block;width:100px;height:100px;border:7px solid #87ceeb3b;border-radius:50%;border-top-color:#4682b4;animation:1s linear infinite spin}@keyframes spin{to{-webkit-transform:rotate(360deg)}}fieldset{padding:0;margin:0;border:0}.toggle{display:none}.lbl-toggle{display:block;font-size:1rem;text-align:left;padding:2px 0 2px 10px;background:#eaeaea;cursor:pointer;border-radius:5px;transition:.25s ease-out}.lbl-toggle:hover{color:#464646}.lbl-toggle::before{content:' ';display:inline-block;border-top:5px solid transparent;border-bottom:5px solid transparent;border-left:5px solid currentColor;vertical-align:middle;margin-right:.5rem;transform:translateY(-2px);transition:transform .2s ease-out}.toggle:checked+.lbl-toggle::before{transform:rotate(90deg) translateX(-3px)}.collapsible-content{max-height:0;overflow:hidden;transition:max-height .25s ease-in-out}.toggle:checked+.lbl-toggle+.collapsible-content{max-height:100vh}.toggle:checked+.lbl-toggle{border-bottom-right-radius:0;border-bottom-left-radius:0}.collapsible-content .content-inner{background:#f3f3f3;border-bottom:1px solid #eaeaea;border-bottom-left-radius:7px;border-bottom-right-radius:7px;padding:.5rem 1rem}.profile-container{display:flex;gap:5px;align-items:center;width:calc(100% + 2px)}.profile-container input{width:calc(50% - 2.5px);margin:0}.profile-container .profile{font-weight:700;color:#666}
 </style>
 )rawliteral";
 #endif


#ifndef HTML_MINIFY
const char webSettingsScript[] PROGMEM = R"rawliteral(
<script>
function btnClick(btn) {
  var xhttp = new XMLHttpRequest();
  xhttp.open('POST','?'+btn+'=',true);
  xhttp.timeout=5000;
  xhttp.send();
}

function urlencode(str) {
  str = (str + '').toString();
  return encodeURIComponent(str)
    .replace('!', '%21')
    .replace('\'', '%27')
    .replace('(', '%28')
    .replace(')', '%29')
    .replace('*', '%2A')
    .replace('%20', '+');
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
   document.execCommand('copy');
   document.body.removeChild(el);
}
</script>
)rawliteral";
#else
const char webSettingsScript[] PROGMEM = R"rawliteral(
<script>
function btnClick(e){var t=new XMLHttpRequest;t.open("POST","?"+e+"=",!0),t.timeout=5e3,t.send()}function urlencode(e){return e=(e+"").toString(),encodeURIComponent(e).replace("!","%21").replace("'","%27").replace("(","%28").replace(")","%29").replace("*","%2A").replace("%20","+")}const collection=document.getElementsByClassName("t1");for(let e=0;e<collection.length;e++){var button=document.createElement("button");button.type="button",button.innerHTML="S",button.className="sb",button.addEventListener("click",(function(t){let n=this.parentNode.previousSibling.firstChild;if(null!=n.nextSibling&&"toggle"===n.nextSibling.className&&(n=this.parentNode.previousSibling.childNodes[5].childNodes[1].childNodes[1]),!1!==n.checkValidity()){var l=n.getAttribute("name"),o=0;if("SELECT"==n.nodeName)o=n.options[n.selectedIndex].value;else if("INPUT"==n.nodeName)"checkbox"==n.type?o=n.checked?1:0:(o=n.value,"fl1"===n.className?o*=10:"fl2"===n.className?o*=100:"fl3"===n.className&&(o*=1e3));else if("FIELDSET"==n.nodeName){var a=n.getElementsByTagName("INPUT");for(l=a[0].getAttribute("name"),e=0;e<a.length;e++)a[e].checked&&(o|=1<<e)}else o=n.value;var c=new XMLHttpRequest;c.onreadystatechange=function(){4==this.readyState&&200==this.status&&alert(this.responseText)},c.open("GET","?SAVE=&"+l+"="+urlencode(o),!0),c.timeout=1e4,c.send()}else alert("Fehler")})),collection[e].appendChild(button)}function copyStringToClipboard(e){var t=document.createElement("textarea");t.value=e,t.setAttribute("readonly",""),t.style={position:"absolute",left:"-9999px"},document.body.appendChild(t),t.select(),document.execCommand("copy"),document.body.removeChild(t)}document.getElementById("lc").style.display="none";
</script>
)rawliteral";
#endif


#ifndef HTML_MINIFY
const char webSettingsScript_Timer[] PROGMEM = R"rawliteral(
<script>
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
  xhttp.open('GET', '%s', true);
  xhttp.timeout=1000;
  xhttp.send();
  var timer = window.setTimeout('getData()', %i);
}
getData();
</script>
)rawliteral";
#else
const char webSettingsScript_Timer[] PROGMEM = R"rawliteral(
<script>
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange=function(){if(4==this.readyState&&200==this.status){let e=this.responseText.split("|");if(1==e.length)document.getElementById("data_div").innerHTML=this.responseText;else for(let t=0;t<e.length;t++){let n=e[t].split(";");"display"==n[0]?document.getElementById("lc").style.display=n[1]:"btn1"==n[0]?"0"==n[1]?document.getElementById("btn1").disabled=!0:document.getElementById("btn1").disabled=!1:document.getElementById(n[0]).innerHTML=n[1]+""}}};
  xhttp.open('GET', '%s', true);xhttp.timeout=1000;xhttp.send();
  var timer = window.setTimeout('getData()', %i);
}
getData();
</script>
)rawliteral";
#endif