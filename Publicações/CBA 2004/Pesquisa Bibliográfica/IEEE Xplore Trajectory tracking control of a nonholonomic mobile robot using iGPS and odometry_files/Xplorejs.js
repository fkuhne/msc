function preprint()
{
            var qry1 = document.VSearch.query1.value;
            qry1 = qry1.toLowerCase();
	        document.VSearch.query1.value = qry1;
    	    var qry2 = document.VSearch.query2.value;
            qry2 = qry2.toLowerCase();
	        document.VSearch.query2.value = qry2;
    	    var qry3 = document.VSearch.query3.value;
            qry3 = qry3.toLowerCase();
	        document.VSearch.query3.value = qry3;
      if(document.VSearch.collection[0].checked ==true)
	  {
	        document.VSearch.collection[5].value = "pprint";  
	  }
      if(document.VSearch.collection[0].checked ==false)
	  {
	       document.VSearch.collection[5].value = " ";
	  }

}
function Advprint()
{
          var qry1 = document.Search.queryText.value;
          qry1 = qry1.toLowerCase();
	      document.Search.queryText.value = qry1;
	   
    if(document.Search.collection[0].checked ==true)
	{
	   document.Search.collection[5].value = "pprint";
	}
    if(document.Search.collection[0].checked ==false)
	{
	   document.Search.collection[5].value = " ";
	}
}

function Scheck()
{
    document.search.sfindtitle.size=30;
    document.search.sfindtitle1.size=30;
    var agt=navigator.userAgent.toLowerCase()
     // *** BROWSER VERSION ***
    this.nav  = ((agt.indexOf('mozilla')!=-1) && ((agt.indexOf('spoofer')==-1)
                && (agt.indexOf('compatible') == -1)))
    this.navonly = (this.nav && (agt.indexOf(";nav") != -1))
    this.ie = (agt.indexOf("msie") != -1)
    if (this.ie){
     		document.search.sfindtitle1.size=43;
            document.search.sfindtitle.size=43;
	}    
    return;
}

function JCcheck()
{

   document.search.findtitle.size=30;
    var agt=navigator.userAgent.toLowerCase()
     // *** BROWSER VERSION ***
    this.nav  = ((agt.indexOf('mozilla')!=-1) && ((agt.indexOf('spoofer')==-1)
                && (agt.indexOf('compatible') == -1)))
    this.navonly = (this.nav && (agt.indexOf(";nav") != -1))
    this.ie = (agt.indexOf("msie") != -1)
    if (this.ie){
     		document.search.findtitle.size=43;
	}    
    return;
}

function locateurl()
{
  var loc_url;
  loc_url = document.location.toString();
  loc_url = loc_url + "&LPChangeStyle=xplorePrint";       
  return loc_url;
}


function Acheck()
{
    document.search.key.size=30;
    var agt=navigator.userAgent.toLowerCase()
     // *** BROWSER VERSION ***
    this.nav  = ((agt.indexOf('mozilla')!=-1) && ((agt.indexOf('spoofer')==-1)
                && (agt.indexOf('compatible') == -1)))
    this.navonly = (this.nav && (agt.indexOf(";nav") != -1))
    this.ie = (agt.indexOf("msie") != -1)
    if (this.ie){
     		document.search.findtitle.size=43;
	}    
    return;
}

function locatequery()
{
      
   if(!document.Search.query1.value)	
	{
           alert("Please enter Search text");
	  return false;
        }
      else{
           var qstr = "((" + document.Search.query1.value + ")and("+document.Search.puNumber.value +"<in>punumber))";
	  document.Search.queryText.value = qstr;
	  return true; 

        }
}

function removequote(oldstr)

{
    oldstr = "%" + oldstr + "%";
	nLoc = 0;

      	nStart = 0;

        var  tempstr;

	tempstr = "'";

	nLoc = oldstr.indexOf(tempstr,nStart);

	sTemp = "";

	while(nLoc != -1)

	{

		sTemp += oldstr.substring(nStart,nLoc);

		sTemp += "''";

		nStart = nLoc+1;

		nLoc = oldstr.indexOf(tempstr,nStart);

	}

	sTemp += oldstr.substring(nStart,oldstr.length);

	return sTemp;
}


function removedash(d)

{

	nLoc = 0;

      	nStart = 0;

        var  tempstr;

	tempstr = '_';

	nLoc = d.indexOf(tempstr,nStart);

	sTemp = "";

	while(nLoc != -1)

	{

		sTemp += d.substring(nStart,nLoc);

		sTemp += " ";

		nStart = nLoc+1;

		nLoc = d.indexOf(tempstr,nStart);

	}

	sTemp += d.substring(nStart,d.length);

	return sTemp;
}


function messagepage()
{
      alert( "Information is not available at this time.");
      
}

function Srch()
{
    var agt=navigator.userAgent.toLowerCase()
     // *** BROWSER VERSION ***
    this.nav  = ((agt.indexOf('mozilla')!=-1) && ((agt.indexOf('spoofer')==-1)
                && (agt.indexOf('compatible') == -1)))
    this.navonly = (this.nav && (agt.indexOf(";nav") != -1))
    this.ie = (agt.indexOf("msie") != -1)
    if (this.ie){
     		    document.VSearch.query1.size=20;
		    document.VSearch.query2.size=20;
		    document.VSearch.query3.size=20;
	}  
    return;
}

function AdvSrch()
{
    var agt=navigator.userAgent.toLowerCase()
     // *** BROWSER VERSION ***
    this.nav  = ((agt.indexOf('mozilla')!=-1) && ((agt.indexOf('spoofer')==-1)
                && (agt.indexOf('compatible') == -1)))
    this.navonly = (this.nav && (agt.indexOf(";nav") != -1))
    this.ie = (agt.indexOf("msie") != -1)
    if (this.ie){
     		    document.Search.queryText.cols=30;
		}  
    return;
}

var	strWinOptions="location=no";
	strWinOptions+=",titlebar=no";
	strWinOptions+=",toolbar=no";
	strWinOptions+=",menubar=no";
	strWinOptions+=",status=no";
	strWinOptions+=",scrollbars=yes";
	strWinOptions+=",resizable=no";
	strWinOptions+=",top=0";
	strWinOptions+=",left=0";
	strWinOptions+=",width=425";
	strWinOptions+=",height=530";
	

function loadwindow(strUrl)
{
	window.open(strUrl,"",strWinOptions);
}

var	strHelpOptions="location=no";
	strHelpOptions+=",toolbar=no";
	strHelpOptions+=",menubar=no";
	strHelpOptions+=",status=no";
	strHelpOptions+=",scrollbars=yes";
	strHelpOptions+=",resizable=yes";
	strHelpOptions+=",top=0";
	strHelpOptions+=",left=0";
	strHelpOptions+=",width=320";
	strHelpOptions+=",height=400";

function loadhelp(strUrl)
{
	window.open(strUrl, "Help", strHelpOptions);
}

var	strfOptions="location=yes";
	strfOptions+=",toolbar=yes";
	strfOptions+=",menubar=yes";
	strfOptions+=",status=yes";
	strfOptions+=",scrollbars=yes";
	strfOptions+=",resizable=yes";
	strfOptions+=",top=50";
	strfOptions+=",left=50";
	strfOptions+=",width=650";
	strfOptions+=",height=650";

function loadfback(fUrl)
{
	window.open(fUrl, "blank", strfOptions);
}


function MM_jumpMenu(targ,selObj,restore){ //v3.0
  eval(targ+".location='"+selObj.options[selObj.selectedIndex].value+"'");
  if (restore) selObj.selectedIndex=0;
}

function MM_swapImgRestore() { //v3.0
  var i,x,a=document.MM_sr; for(i=0;a&&i<a.length&&(x=a[i])&&x.oSrc;i++) x.src=x.oSrc;
}

function MM_preloadImages() { //v3.0
  var d=document; if(d.images){ if(!d.MM_p) d.MM_p=new Array();
    var i,j=d.MM_p.length,a=MM_preloadImages.arguments; for(i=0; i<a.length; i++)
    if (a[i].indexOf("#")!=0){ d.MM_p[j]=new Image; d.MM_p[j++].src=a[i];}}
}

function MM_findObj(n, d) { //v4.0
  var p,i,x;  if(!d) d=document; if((p=n.indexOf("?"))>0&&parent.frames.length) {
    d=parent.frames[n.substring(p+1)].document; n=n.substring(0,p);}
  if(!(x=d[n])&&d.all) x=d.all[n]; for (i=0;!x&&i<d.forms.length;i++) x=d.forms[i][n];
  for(i=0;!x&&d.layers&&i<d.layers.length;i++) x=MM_findObj(n,d.layers[i].document);
  if(!x && document.getElementById) x=document.getElementById(n); return x;
}

function MM_swapImage() { //v3.0
  var i,j=0,x,a=MM_swapImage.arguments; document.MM_sr=new Array; for(i=0;i<(a.length-2);i+=3)
   if ((x=MM_findObj(a[i]))!=null){document.MM_sr[j++]=x; if(!x.oSrc) x.oSrc=x.src; x.src=a[i+2];}
}

function getAccess (name) {
var mtcookie = document.cookie; 
var cname = name + "=";
var clen = mtcookie.length;
var cbegin = 0;
        while (cbegin < clen) {
        var vbegin = cbegin + cname.length;
                if (mtcookie.substring(cbegin, vbegin) == cname) { 
                var vend = mtcookie.indexOf (";", vbegin);
                        if (vend == -1) vend = clen;
                return unescape(mtcookie.substring(vbegin, vend));
                }
        cbegin = mtcookie.indexOf(" ", cbegin) + 1;
                if (cbegin == 0) break;
        }
return;
}


function openurl()
{
     
var s = getAccess("ERIGHTS");;	
	//alert(s);

if(!s)
{
  s = "NotSet";
}

if(s == "NotSet"){
	
	top.location='javascript:loadfback("/Xplore/Sublogin.jsp")';
        }else{
	top.location = 'javascript:loadfback("/Xplore/routeSub.jsp")';
        }
}
function removespace(s)

{
	strp = new String();
	strp1 = new String();
	strp = s;
	strp1 = s;
      chp = " ";
      nStart=0; 
	var pos = strp.lastIndexOf(chp);
        if (pos>=0)
			{
 			  	strp = strp.substring(nStart,pos) + "%20";
				nStart = pos + 1;
				strp += strp1.substring(nStart, strp1.length);
			}
	return strp;

}
var	strmmOptions="location=no";
	strmmOptions+=",toolbar=no";
	strmmOptions+=",menubar=no";
	strmmOptions+=",status=no";
	strmmOptions+=",scrollbars=yes";
	strmmOptions+=",resizable=yes";
	strmmOptions+=",top=0";
	strmmOptions+=",left=0";
	strmmOptions+=",width=550";
	strmmOptions+=",height=550";

function mmfiles(strUrl)
{
	window.open(strUrl, "Multimedia", strmmOptions);
}
function GenerateindxUrl(inx)

{

      idxUrls="";
      var indx = convertToVerity(inx);
      
      idxUrls="/search97/s97is.vts?Action=Search&ResultTemplate=index_cit.hts&ViewTemplate=lpdocview.hts&querytext='"+ indx + 	"'<in>de&ResultCount=15&SortField=pyr&SortOrder=desc";
      top.location = idxUrls;     
}
function fixperiod(s)
{
	nLoc = 0;
        nStart = 0;
	nLoc = s.indexOf(".",nStart);
	sTemp = "";
	while(nLoc != -1)
	{
		sTemp += s.substring(nStart,nLoc);
		sTemp += ".%20";
		nStart = nLoc +1;
		nLoc = s.indexOf(".",nStart);
	}
	sTemp += s.substring(nStart,s.length);
	return sTemp;

}
function convertToVerity(sString)
{
	sReplace = new Object();
	sReplace[0] = ",";
	sReplace[1] = "'";
	sReplace[2] = ";";
	sReplace[3] = ":";
	sReplace[4] = "_";
	sReplace[5] = "(";
	sReplace[6] = ")";
	sReplace[7] = "[";
	sReplace[8] = "]";
	sReplace[9] = "{";
	sReplace[40] = "}";
	sReplace[11] = "-";
	sReplace[12] = "/";
	sReplace[13] = "&";
	sReplace[14] = "?";
	sReplace[15] = " ";
	sTemp1 = new String();
	sTempHold = new String();
	sTempHold = sString;
        for(x in sReplace)
	{
		pos = sTempHold.indexOf(sReplace[x]);
		if(pos >=0)
		{
			nStart=0;
			while(pos >= 0 )
			{
				sTemp1 += sTempHold.substring(nStart,pos) + "%20";
				nStart = pos + 1;
				pos = sTempHold.indexOf(sReplace[x], nStart);
			}
			sTemp1 += sTempHold.substring(nStart,sTempHold.length);
			sTempHold = sTemp1;
			sTemp1 = "";
		}
	}
  

 pos = sTempHold.toLowerCase().indexOf("%20and%20");
 sTemp1 = ""; 
 if(pos >= 0)
 {
  nStart = 0;
  while(pos>=0)
  {
   sTemp1 += sTempHold.substring(nStart,pos) + "%20'and'%20";
   nStart = pos + 9;
   pos = sTempHold.toLowerCase().indexOf("%20and%20", nStart);
  }
  sTemp1 += sTempHold.substring(nStart, sTempHold.length);
  sTempHold = sTemp1;
 }

 pos = sTempHold.toLowerCase().indexOf("%20or%20");
 sTemp1 = ""; 
 if(pos >= 0)
 {
  nStart = 0;
  while(pos>=0)
  {
   sTemp1 += sTempHold.substring(nStart,pos) + "%20'or'%20";
   nStart = pos + 8;
   pos = sTempHold.toLowerCase().indexOf("%20or%20", nStart);
  }
  sTemp1 += sTempHold.substring(nStart, sTempHold.length);
  sTempHold = sTemp1;
 }

 return sTempHold;
}
