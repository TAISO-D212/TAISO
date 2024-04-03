```html
<html>
<head>
  <title>TAISO Porting 매뉴얼</title>
  <style>
    
:root {
  --default-font: ui-sans-serif, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif, "Apple Color Emoji", "Segoe UI Emoji", "Segoe UI Symbol", "Microsoft YaHei Light", sans-serif;
  --font-monospace: 'Source Code Pro', monospace;
  --background-primary: #ffffff;
  --background-modifier-border: #ddd;
  --text-accent: #705dcf;
  --text-accent-hover: #7a6ae6;
  --text-normal: #2e3338;
  --background-secondary: #f2f3f5;
  --background-secondary-alt: #e3e5e8;
  --text-muted: #888888;
  --font-mermaid: ui-sans-serif, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Inter", "Apple Color Emoji", "Segoe UI Emoji", "Segoe UI Symbol", "Microsoft YaHei Light", sans-serif;
  --text-error: #E4374B;
  --background-primary-alt: '#fafafa';
  --background-accent: '';
  --interactive-accent: hsl( 254,  80%, calc( 68% + 2.5%));
  --background-modifier-error: #E4374B;
}

    body,input {
  font-family: "Roboto","Helvetica Neue",Helvetica,Arial,sans-serif
}

code, kbd, pre {
  font-family: "Roboto Mono", "Courier New", Courier, monospace;
  background-color: #f5f5f5;
}

pre {
  padding: 1em 0.5em;
}

table {
  background: white;
  border: 1px solid #666;
  border-collapse: collapse;
  padding: 0.5em;
}

table thead th,
table tfoot th {
  text-align: left;
  background-color: #eaeaea;
  color: black;
}

table th, table td {
  border: 1px solid #ddd;
  padding: 0.5em;
}

table td {
  color: #222222;
}

.callout[data-callout="abstract"] .callout-title,
.callout[data-callout="summary"] .callout-title,
.callout[data-callout="tldr"]  .callout-title,
.callout[data-callout="faq"] .callout-title,
.callout[data-callout="info"] .callout-title,
.callout[data-callout="help"] .callout-title {
  background-color: #828ee7;
}
.callout[data-callout="tip"] .callout-title,
.callout[data-callout="hint"] .callout-title,
.callout[data-callout="important"] .callout-title {
  background-color: #34bbe6;
}
.callout[data-callout="success"] .callout-title,
.callout[data-callout="check"] .callout-title,
.callout[data-callout="done"] .callout-title {
  background-color: #a3e048;
}
.callout[data-callout="question"] .callout-title,
.callout[data-callout="todo"] .callout-title {
  background-color: #49da9a;
}
.callout[data-callout="caution"] .callout-title,
.callout[data-callout="attention"] .callout-title {
  background-color: #f7d038;
}
.callout[data-callout="warning"] .callout-title,
.callout[data-callout="missing"] .callout-title,
.callout[data-callout="bug"] .callout-title {
  background-color: #eb7532;
}
.callout[data-callout="failure"] .callout-title,
.callout[data-callout="fail"] .callout-title,
.callout[data-callout="danger"] .callout-title,
.callout[data-callout="error"] .callout-title {
  background-color: #e6261f;
}
.callout[data-callout="example"] .callout-title {
  background-color: #d23be7;
}
.callout[data-callout="quote"] .callout-title,
.callout[data-callout="cite"] .callout-title {
  background-color: #aaaaaa;
}

.callout-icon {
  flex: 0 0 auto;
  display: flex;
  align-self: center;
}

svg.svg-icon {
  height: 18px;
  width: 18px;
  stroke-width: 1.75px;
}

.callout {
  overflow: hidden;
  margin: 1em 0;
  box-shadow: 0 2px 2px 0 rgba(0, 0, 0, 0.14), 0 1px 5px 0 rgba(0, 0, 0, 0.12), 0 3px 1px -2px rgba(0, 0, 0, 0.2);
  border-radius: 4px;
}

.callout-title {
  padding: .5em;
  display: flex;
  gap: 8px;
  font-size: inherit;
  color: black;
  line-height: 1.3em;
}

.callout-title-inner {
  font-weight: bold;
  color: black;
}

.callout-content {
  overflow-x: auto;
  padding: 0.25em .5em;
  color: #222222;
  background-color: white !important;
}

ul.contains-task-list {
  padding-left: 0;
  list-style: none;
}

ul.contains-task-list ul.contains-task-list {
  padding-left: 2em;
}

ul.contains-task-list li input[type="checkbox"] {
  margin-right: .5em;
}

.callout-table,
.callout-table tr,
.callout-table p {
  width: 100%;
  padding: 0;
}

.callout-table td {
  width: 100%;
  padding: 0 1em;
}

.callout-table p {
  padding-bottom: 0.5em;
}

.source-table {
  width: 100%;
  background-color: #f5f5f5;
}

  </style>
</head>
<body>
<div><h2 data-heading="I. 개요">I. 개요</h2>
<hr>
<h1 data-heading="<span style=&quot;color:gray;background-color:#E6E6FA&quot;> 자율주행 시골버스 TAISO</span>"><span style="color:gray;background-color:#E6E6FA"> 자율주행 시골버스 TAISO</span></h1>
<p><img src="https://i.imgur.com/rYL1KGC.png" referrerpolicy="no-referrer"></p>
<h5 data-heading="1. 프로젝트 개요">1. 프로젝트 개요</h5>
<div data-callout-metadata="" data-callout-fold="" data-callout="앱으로-호출하는-편리한-시골-택시" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">앱으로 호출하는 편리한 시골 택시</div></div><div class="callout-content">
<pre><code>→  터치 한번으로 예약하는 편리함!  
→  교통 소외지역에 이동의 자유를!
</code></pre>
</div></div>
<div data-callout-metadata="" data-callout-fold="" data-callout="합승-서비스로-경제적인-주행-제공" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">합승 서비스로 경제적인 주행 제공</div></div><div class="callout-content">
<pre><code> →  마을 사람들의 예약 목록을 확인하여 합승 신청!
 →  효율적인 주행으로 불필요한 이동 및 대기 시작 감소!
</code></pre>
</div></div>
<div data-callout-metadata="" data-callout-fold="" data-callout="실시간-이동경로-확인" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">실시간 이동경로 확인</div></div><div class="callout-content">
<pre><code> →  현재 이동 중인 위치를 실시간으로 확인!
 →  예상 도착시간 확인 가능!
</code></pre>
</div></div>
<h5 data-heading="2. 프로젝트 목표 및 배경">2. 프로젝트 목표 및 배경</h5>
<div data-callout-metadata="" data-callout-fold="" data-callout="목표" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">목표</div></div><div class="callout-content">
<ul>
<li><strong>교통 소외지역 주민의 이동권 보장을 위한 자율주행 서비스 제공</strong>
<ul>
<li>도심지에 비해 부족한 시골 지역의 교통 인프라를 자율 주행 버스 서비스를 이용해 개선</li>
<li>부족한 인력과 수익 구조를 무인 자율화를 통해 보완</li>
</ul>
</li>
<li><strong>지역소멸 위기에 처한 시골 지역에 인구 유치 및 균형 발전 도모</strong></li>
<li><strong>이동이 불편한 노령 인구를 직접 이송함으로써 노인의 이동 편의성 증대 및 교류 확대</strong></li>
</ul>
</div></div>
<div data-callout-metadata="" data-callout-fold="" data-callout="배경" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">배경</div></div><div class="callout-content">
<ul>
<li><strong>디지털 소외계층에 대한 사용자 경험 기반 서비스 제공</strong>
<ul>
<li>예전부터 존재했던 버스와 자율주행을 결합한 새로운 형태의 서비스</li>
</ul>
<ul>
<li>농촌지역에 도입되는 자율주행기반 기술 체험과 인식 개선</li>
</ul>
</li>
</ul>
<ul>
<li><strong>농촌 주민의 소외된 삶, 사회적 배려</strong></li>
<li><strong>도심지에 집중된 인프라 보완</strong></li>
</ul>
<ul>
<li><strong> 자율주행 기반으로 교통복지 예산 낭비 방지</strong></li>
</ul>
</div></div>
<h5 data-heading="3. 기대 효과">3. 기대 효과</h5>
<div data-callout-metadata="" data-callout-fold="" data-callout="기대효과" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">기대효과</div></div><div class="callout-content">
<ul>
<li><strong>교통 소외지역 주민의 사회 참여 증가</strong>
<ul>
<li>시골 지역의 교통 인프라 개선</li>
<li>자율주행 버스가 교통 소외지역 주민에게 편의성 제공</li>
<li>교통 소외지역 주민의 교류와 이동 증가로 교통 소외 지역 주민의 사회 참여 증가</li>
</ul>
</li>
</ul>
<ul>
<li><strong>지역 경제 활성화</strong>
<ul>
<li>지역에 새로운 이동 수단이 생기면서 소비 활동이 증가하고, 지역 상권의 활성</li>
<li>관광객들의 접근성이 증대되면서 지역 관광 수입 증가 기대</li>
<li>자율주행 버스를 통해 도심 이외의 지역에도 효율적인 교통 수단을 제공함으로써, 도시와 농촌 간의 격차 해소</li>
<li>자율주행 버스 도입을 통한 노인의 경제활동 참여 증가</li>
</ul>
</li>
</ul>
</div></div>

<h5 data-heading="4. 팀원 소개">4. 팀원 소개</h5>
- 삼성 청년 SW 아카데미 SSAFY 10기 2학기 특화 프로젝트 구미 2반 12팀
- 2023.2.19 ~ 2023.4.5
<div data-callout-metadata="" data-callout-fold="" data-callout="팀원-소개" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">팀원 소개</div></div><div class="callout-content">
<p>Front-End</p>
<ul>
<li>김태용</li>
<li>전근렬</li>
</ul>
<p>Back-End</p>
<ul>
<li>전근렬</li>
<li>배성연</li>
<li>정경리</li>
</ul>
<p>자율주행</p>
<ul>
<li>전인구</li>
<li>양원석</li>
</ul>
<p> 인프라</p>
<ul>
<li>정경리</li>
</ul>
</div></div>


<h2 data-heading="II. 기술 스택 &amp; 개발 환경">II. 기술 스택 &amp; 개발 환경</h2>
<hr>
<div data-callout-metadata="" data-callout-fold="" data-callout="사용-도구" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">사용 도구</div></div><div class="callout-content">
<ul>
<li>이슈 관리: JIRA</li>
<li>형상 관리: GitLab</li>
<li>커뮤니케이션: Mattermost, Notion, Google Docs</li>
<li>디자인: Figma, Canva</li>
<li>UCC: Movavi</li>
<li>CI/CD: EC2, Docker, Jenkins</li>
</ul>
</div></div>
<div data-callout-metadata="" data-callout-fold="" data-callout="개발-환경" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">개발 환경</div></div><div class="callout-content">
<ul>
<li><strong>Front-end</strong>
<ul>
<li>Node.js: 20.11.1</li>
<li>React: 18.2.0</li>
<li>Typescript: 5.2.2</li>
<li>ESLint: 7.1.1</li>
<li>Prettier: 3.2.5</li>
<li>DaisyUI: 4.9.0</li>
<li>PWA: 0.19.7</li>
</ul>
</li>
<li><strong>Back-end</strong>
<ul>
<li>JDK: 17.0.10 LTS</li>
<li>SpringBoot: 3.2.3</li>
<li>SpringSecurity</li>
<li>Gradle: 8.5</li>
<li>jjwt-api:0.11.5</li>
<li>MQTT: spring-integration-mqtt 5.5.0</li>
<li>Firebase Admin SDK: firebase-admin:9.2.0</li>
<li>DotEnv: dotenv-java:3.0.0</li>
<li>Swagger: 2.3.0</li>
<li>Mapper: 1.5.3.Final</li>
</ul>
</li>
<li><strong>DB</strong>
<ul>
<li>MariaDB: 10.11.7 LTS</li>
</ul>
</li>
<li><strong>Self-driving</strong>
<ul>
<li>Simulator : MORAI SIM ver22.R2.1</li>
<li>Python : 3.8.10
<ul>
<li>scikit-learn : 1.3.1</li>
<li>scipy : 1.10.1</li>
</ul>
</li>
<li>Linux os : Ubuntu 20.04.6 LTS</li>
<li>ROS : noetic</li>
<li>NVDIA Driver : 470.199.02</li>
<li>CUDA Version: 11.4</li>
</ul>
</li>
<li><strong>Infra</strong>
<ul>
<li>Server:
<ul>
<li>AWS EC2: Ubuntu 20.04.6 LTS</li>
</ul>
</li>
<li>Containerization Platform:
<ul>
<li>Docker: 25.0.4</li>
</ul>
</li>
<li>CI/CD:
<ul>
<li>Jenkins: 2.448</li>
</ul>
</li>
<li>Web Server:
<ul>
<li>Nginx: 1.18.0 (Ubuntu)</li>
</ul>
</li>
</ul>
</li>
</ul>
</div></div>
<div data-callout-metadata="" data-callout-fold="" data-callout="외부-서비스" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">외부 서비스</div></div><div class="callout-content">
<ul>
<li>Kakao Map API</li>
<li>Firebase API</li>
<li>Morai Simulator</li>
</ul>
</div></div>
<div data-callout-metadata="" data-callout-fold="" data-callout="gitignore" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">Gitignore</div></div></div>
<pre><code>[Back]

HELP.md  
.gradle  
build/  
!gradle/wrapper/gradle-wrapper.jar  
!**/src/main/**/build/  
!**/src/test/**/build/  
  
### STS ###  
.apt_generated  
.classpath  
.factorypath  
.project  
.settings  
.springBeans  
.sts4-cache  
bin/  
!**/src/main/**/bin/  
!**/src/test/**/bin/  
  
### IntelliJ IDEA ###  
.idea  
*.iws  
*.iml  
*.ipr  
out/  
!**/src/main/**/out/  
!**/src/test/**/out/  
  
### NetBeans ###  
/nbproject/private/  
/nbbuild/  
/dist/  
/nbdist/  
/.nb-gradle/  
  
### VS Code ###  
.vscode/  
  
### FCM 비공개 키  ###
/src/main/resources/{FCM 비공개 키 파일 이름}.json
</code></pre>
<pre><code>[Front]

### Logs ###
logs
*.log
npm-debug.log*
yarn-debug.log*
yarn-error.log*
pnpm-debug.log*
lerna-debug.log*

node_modules
dist
dist-ssr
*.local

# Editor directories and files
.vscode/*
!.vscode/extensions.json
.idea
.DS_Store
*.suo
*.ntvs*
*.njsproj
*.sln
*.sw?
</code></pre>
<div data-callout-metadata="" data-callout-fold="" data-callout="환경변수-(front)" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">환경변수 (front)</div></div></div>
<pre><code>VITE_BASE_URL={사이트 URL}
VITE_KAKAO_MAP_API_KEY={카카오맵 API 키}

##### Firebase 프로젝트를 식별하는 데 사용
VITE_FIREBASE_API_KEY={Firebase 프로젝트의 API 키}

##### Firebase Authentication 서비스를 사용하여 인증할 때 사용
VITE_FIREBASE_AUTH_DOMAIN={Firebase Authentication 도메인}

##### Firebase 콘솔에서 프로젝트를 식별하는 데 사용
VITE_FIREBASE_PROJECT_ID={Firebase 프로젝트의 ID}

##### Firebase Storage 서비스에서 파일을 저장할 때 사용
VITE_FIREBASE_STORAGE_BUCKET={Firebase Storage 버킷}

##### Firebase Cloud Messaging (FCM)에서 사용
VITE_FIREBASE_MESSAGING_SENDER_ID={Firebase 프로젝트의 메시징 발신자 ID}

##### Firebase 앱을 식별하는 데 사용
VITE_FIREBASE_APP_ID={Firebase 앱의 ID}

##### Firebase 애널리틱스 서비스를 사용하여 앱 이벤트를 추적할 때 사용
VITE_FIREBASE_MEASUREMENT_ID={Firebase 애널리틱스의 측정 ID}

##### 웹 앱에서 Firebase Cloud Messaging(FCM)을 사용하여 푸시 알림을 보낼 때 사용
VITE_FIREBASE_PUBLIC_VAPID_KEY={Firebase 웹 푸시 알림을 위한 VAPID 키}
</code></pre>
<pre><code>[Back]
# 서버 설정
SERVER_PORT={백엔드 서버용 포트}

##### DB 서버 연결 설정
DB_URL=jdbc:mariadb://{도메인}:{DB용 포트}/{DB 이름}
DB_USERNAME={DB 아이디}  
DB_PASSWORD={DB 패스워드}

##### MQTT 연결 설정
MQTT_BROKER_URL=tcp://{도메인}:{MQTT용 포트}
</code></pre>
<h2 data-heading="III. 빌드 및 배포">III. 빌드 및 배포</h2>
<hr>
<div data-callout-metadata="" data-callout-fold="" data-callout="개발-환경에서-직접-빌드(로컬-빌드)" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">개발 환경에서 직접 빌드(로컬 빌드)</div></div><div class="callout-content">
<p>[Front]</p>
<ol>
<li>
<p>의존성 설치<br>
<code>npm install</code></p>
</li>
<li>
<p>프로젝트 빌드 (정적 파일 생성)<br>
<code>npm run build</code></p>
</li>
</ol>
<p>[Back]</p>
<ol>
<li>
<p>프로젝트 빌드<br>
<code>./gradlew build</code><br>
(Gradle - Tasks - build - bootJar 로도 jar 파일 생성 가능)</p>
</li>
<li>
<p>빌드된 JAR 파일 실행<br>
<code>java -jar build/libs/{프로젝트명}.jar</code></p>
</li>
</ol>
<ul>
<li>빌드/실행 동시에 하는 경우<br>
<code>./gradlew bootRun</code></li>
</ul>
</div></div>
<div data-callout-metadata="" data-callout-fold="" data-callout="배포-시-빌드(jenkins-파이프라인)" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">배포 시 빌드(jenkins 파이프라인)</div></div></div>
<blockquote>
<p><strong>jenkins 파이프라인</strong></p>
</blockquote>
<pre><code>pipeline {
    agent any

    environment {
        // BE 디렉터리명
        DIR_BE = 'BE/taiso'
        // FE 디렉터리명
        DIR_FE = 'FE'
        IMG_FE = 'fe-img'
        IMG_BE = 'be-img'
        CONT_FE = 'TAISO-fe'
        CONT_BE = 'TAISO-be'
        FE_PATH = '/TAISO/Front'
        DOCKERFILE_BE = '/TAISO/Back'
        PROJECT_NAME = 'Taiso-develop'
    }
	
    stages {
        
        //다운받기 전에 이전 폴더 삭제하기
        stage('Remove Previous Diretory') {
            steps {
                          
                sh 'rm -rf ${PROJECT_NAME}'
            }
        }
        
        stage('Checkout') {
            steps {
                git credentialsId: '7f65973d-a3e4-4815-9bd5-09326151d5cd',
                url: 'https://lab.ssafy.com/s10-mobility-autodriving-sub2/S10P22D212.git',
                branch: 'develop' // 원하는 브랜치 지정
                script {
                    def currentDir = pwd()
                    echo "Current Directory: ${currentDir}"
                }
            }
        }
        
        stage('Add Env') {
            steps {
                dir("${DIR_BE}") {
                    withCredentials([file(credentialsId: 'env', variable: 'env')]) {
                      sh "cp \$env .env"
                      sh "chmod +r .env"
                    }
                }
            }
        }
        
        stage('Add FCM SDK KEY') {
            steps {
                dir("${DIR_BE}/src/main/resources") {
                    withCredentials([file(credentialsId: 'fcmKEY', variable: 'fcmKEY')]) {
                      sh "cp \$fcmKEY taiso-18ea8-firebase-adminsdk-m9qcd-e458ff02c9.json"
                      sh "chmod +r taiso-18ea8-firebase-adminsdk-m9qcd-e458ff02c9.json"
                    }
                }
            }
        }
        
		
        stage('Build main BE image') {
            steps {
                
               script {
                try {
                    sh 'ls -al'
                    dir("${DIR_BE}") {
                        sh 'ls -al'
                        sh 'chmod +x ./gradlew'
                        sh './gradlew clean build'
                        sh "docker build . -t ${IMG_BE}"
                        }
                    echo 'Build main image...'
                } catch (Exception e) {
                    
                    // 에러가 발생하면 에러 로그를 파일에 저장
                    def errorLog = "Error occurred in Build main BE image stage:\n${e}\n"
                    writeFile file: 'error.log', text: errorLog
                    currentBuild.result = 'FAILURE' // 빌드를 실패로 표시
                    throw e // 예외를 다시 던져서 빌드를 중단
                    
                    }
                }

            }
        }


        //BE - 이전 컨테이너 삭제
        stage('Remove Previous main BE Container') {
            steps {
                script {
                    try {
                        sh "docker stop ${CONT_BE}"
                        sh "docker rm ${CONT_BE}"
                    } catch (e) {
                        echo 'fail to stop and remove main container'
                    }
                }
            }
        }

      //새 BE 컨테이너 실행
        stage('Run New main BE image') {
            steps {
                sh "docker run --name ${CONT_BE} -d -p 3000:3000 -v /home/ubuntu/docker/jenkins-data/workspace/Taiso-develop/BE/.env:/home/ubuntu/docker/jenkins-data/workspace/Taiso-develop/BE/.env ${IMG_BE}"
                echo 'Run New main BE image'
            }
        }
        
        
        
        //FE - 이미지 생성
        stage('Build FE image') {
            steps {
                dir("${DIR_FE}") {
                    sh "ls"
                    sh "docker build . -t ${IMG_FE}:latest --no-cache"
                    script {
                        def currentDir = pwd()
                        echo "Current Directory: ${currentDir}"
                    }
                }
            }
        }
        
        
                //이전 컨테이너 삭제
        stage('Remove Previous Container') {
            steps {
                script {
                    try {
                        sh "docker stop ${CONT_FE}"
                        sh "docker rm ${CONT_FE}"
                    } catch (e) {
                        echo 'fail to stop and remove container'
                    }
                }
            }
        }
        
        //새 FE 컨테이너 실행
        stage('Run FE image') {
            steps {
                script {
                    sh "docker run --name ${CONT_FE} -d -p 5173:5173 ${IMG_FE}:latest"
                    sh "docker cp TAISO-fe:/app/dist ."
                }
            }
        }
        

    }
    
        post {
            
            always{
                script{
                    // 빌드 결과에 따라 Mattermost로 메시지 전송
                    def Author_ID = sh(script: "git show -s --pretty=%an", returnStdout: true).trim()
                    def Author_Name = sh(script: "git show -s --pretty=%ae", returnStdout: true).trim()
                    
                    if (currentBuild.result == 'SUCCESS') {
                        // 성공 시
                        mattermostSend (
                            color: 'good', 
                            message: "빌드 성공: ${env.JOB_NAME} #${env.BUILD_NUMBER} by ${Author_ID}(${Author_Name})\n(&lt;${env.BUILD_URL}|Details&gt;)", 
                            endpoint: 'https://meeting.ssafy.com/hooks/6ychb7e6ajnimj5ybt1tzgsg7o', 
                            channel: '212'
                        )
                    } else {
                        // 실패 시
                        def errorLog = readFile('error.log')
                        mattermostSend (
                            color: 'danger', 
                            message: "빌드 실패: ${env.JOB_NAME} #${env.BUILD_NUMBER} by ${Author_ID}(${Author_Name})\n(&lt;${env.BUILD_URL}|Details&gt;)\n에러 로그:\n${errorLog}", 
                            endpoint: 'https://meeting.ssafy.com/hooks/6ychb7e6ajnimj5ybt1tzgsg7o', 
                            channel: '212'
                        )
                    }
                }
            }
        }
            
}

</code></pre>
<blockquote>
<p>Frontend Dockerfile</p>
</blockquote>
<pre><code>FROM node:20.11.1 AS build

##### 작업 디렉토리 설정
WORKDIR /app

##### 소스 코드 복사
COPY . .

##### npm install을 하기 전에 불필요한 파일을 삭제
RUN rm -rf /app/node_modules /app/package-lock.json

##### 필요한 패키지 설치
RUN npm install

##### React 프로젝트 빌드
RUN npm run build
</code></pre>
<p><code>→ Dockerfile을 사용해서 빌드한 후, 젠킨스 컨테이너에서 만들어진 dist 디렉터리를 컨테이너 밖으로 복사해 옴.</code></p>
<blockquote>
<p>Backend Dockerfile</p>
</blockquote>
<pre><code># base 이미지 설정
FROM amazoncorretto:17

##### Docker 컨테이너 안으로 환경변수 파일 복사
COPY .env /.env

##### json
COPY src/main/resources/{fcm 비공개 키 이름}.json src/main/resources/{fcm 비공개 키 이름}.json

##### jar 파일을 컨테이너 내부에 복사
COPY build/libs/{복사 전 jar파일 이름: Taiso(예시)}.jar /{저장할 jar파일 이름: Taiso-copy(예시)}.jar

##### 외부에 호출될 포트 설정
EXPOSE {백엔드 포트번호}

##### 실행 명령어
ENTRYPOINT ["java","-jar","/{저장한 jar파일 이름: Taiso-copy(예시) }.jar"]
</code></pre>
<div data-callout-metadata="" data-callout-fold="" data-callout="nginx-설정파일" class="callout"><div class="callout-title"><div class="callout-icon"><img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAbpJREFUSEu11btqVFEUxvFfTKN4wU5FiffCUkUFY1BI6wsEtfAVxMbGUmx8ABtLX0AsvKCIKAS0sBALbzFEQwoRwSYg6l6yd9gOZ2bOXDwwHIa99/f/1jpnfWfCf74mxqC/HpdwDrvxElfxJLRHBezAnfQ72mB0Fo9GAYR4uDyIRVzAR1zDebzAsWEB2/EBG/L9FJZzFVvwHT+weRhAOH+K/VlwAScrwBQ+4TN2DQrYlsWjLVHBOuxJLXmHGfzGYxxK7m+kKi4PAqh7/jYJnc6CUc0BRCW/sA+vcAKrbQHh/Hk+HM6jJSu5RQF+hr35/z/rbQC9xEMz1ufzDKzme4H3nYM24t0q+1tQrwpGFu8FGIt4N8DYxJsAYxXvBIR4vG4xoe/z4JTxj731BDetNwZz/ZAjpK6k2C1D1Clegq1pvWvq14DI8SM4U7I8n6rjYSDxukVb8S1GG5vwsxLv+Z73+2CVCuZwO+X33ZTjZ7ERx1Pfb+a8D+cRZmsT2k+4rBfArRSxF/EaXzGNybxp4LbU8AJYws4OV2/wENfxpa3jzn0FEBl+GPeS+wcpLe/nz+Cwumvn2qTpSJA/QWJ1GdtkkC4AAAAASUVORK5CYII=" style=""></div><div class="callout-title-inner">Nginx 설정파일</div></div></div>
<pre><code>server {
    listen       80;
    listen [::]:80 default_server;
    server_name  {도메인};
    root         {정적 파일의 경로};
    return 301 https://$host$request_uri;
}

server {
    # TLS1, SSLv2, SSLv3는 보안에 취약하므로 사용하지 말 것.
    ssl_protocols  TLSv1.3 TLSv1.2 TLSv1.1;
	
    listen 443 ssl;
    listen [::]:443 ssl;
	
    # root에 정적 파일이 위치한 디렉토리를 지정함.
    # -&gt; 클라이언트에 제공할 정적 파일들이 위치함.
    root {정적 파일의 경로};
	
    server_name {도메인};
	
    # HTTP 요청을 처리하는 부분
    # 클라이언트로부터의 요청이 /로 시작할 때의 처리를 정의
    location / {
    # 정적 파일을 제공하고, 존재하지 않는 경우에는 /index.html 파일을 반환
        try_files $uri $uri/ /index.html;
    }
	
    location /api/ { # Backend 서버로의 요청을 처리할 엔드포인트
        # 프록시 대상 서버 및 포트 설정
        proxy_pass http://{도메인}:{백엔드 서버 포트};
    }
	
	
     # SSL 설정을 위한 인증서 및 개인 키 파일의 경로를 지정
    ssl_certificate /etc/letsencrypt/live/{도메인}/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/{도메인}/privkey.pem;
    include /etc/letsencrypt/options-ssl-nginx.conf;
    ssl_dhparam /etc/letsencrypt/ssl-dhparams.pem;

}

</code></pre></div>


<h2 data-heading="IV. 프로젝트 산출물">IV. 프로젝트 산출물</h2>
<hr>
<h5 data-heading="[기능 명세서](https://kimtaeyong.notion.site/dbf6a8fdc14d494e8172ca040bb36038)"><a data-tooltip-position="top" aria-label="https://kimtaeyong.notion.site/dbf6a8fdc14d494e8172ca040bb36038" rel="noopener" class="external-link" href="https://kimtaeyong.notion.site/dbf6a8fdc14d494e8172ca040bb36038" target="_blank">기능 명세서</a></h5>
<h5 data-heading="[요구사항 정의서](https://kimtaeyong.notion.site/86156328c3734bf6b6bcb4360d5c81a6)"><a data-tooltip-position="top" aria-label="https://kimtaeyong.notion.site/86156328c3734bf6b6bcb4360d5c81a6" rel="noopener" class="external-link" href="https://kimtaeyong.notion.site/86156328c3734bf6b6bcb4360d5c81a6" target="_blank">요구사항 정의서</a></h5>
<h5 data-heading="[API 명세서](https://kimtaeyong.notion.site/API-23b59c75b0584712b4d625b71ff70efe)"><a data-tooltip-position="top" aria-label="https://kimtaeyong.notion.site/API-23b59c75b0584712b4d625b71ff70efe" rel="noopener" class="external-link" href="https://kimtaeyong.notion.site/API-23b59c75b0584712b4d625b71ff70efe" target="_blank">API 명세서</a></h5>
<h5 data-heading="[ERD](https://kimtaeyong.notion.site/ERD-4e00456b5ad64f2586e0b24d4ff7eb4b)"><a data-tooltip-position="top" aria-label="https://kimtaeyong.notion.site/ERD-4e00456b5ad64f2586e0b24d4ff7eb4b" rel="noopener" class="external-link" href="https://kimtaeyong.notion.site/ERD-4e00456b5ad64f2586e0b24d4ff7eb4b" target="_blank">ERD</a></h5>
<h5 data-heading="아키텍처">아키텍처</h5>
<h5 data-heading="프로젝트 파일 구조">프로젝트 파일 구조</h5>
<h2 data-heading="V. 프로젝트 결과물">V. 프로젝트 결과물</h2>
<hr>
<h5 data-heading="포팅 메뉴얼">포팅 메뉴얼</h5>
<h5 data-heading="중간 발표 자료">중간 발표 자료</h5>
<h5 data-heading="최종 발표 자료">최종 발표 자료</h5>
<h5 data-heading="UCC 영상">UCC 영상</h5>
<h2 data-heading="VI. TAISO 서비스 화면">VI. TAISO 서비스 화면</h2>
<hr>
<h3 data-heading="1. 앱 화면">1. 앱 화면</h3>
<h3 data-heading="2. 시뮬레이터 화면">2. 시뮬레이터 화면</h3></div>


</body>
</html>
```