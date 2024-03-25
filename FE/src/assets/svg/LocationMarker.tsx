import type { SVGProps } from 'react';
const SvgLocationMarker = (props: SVGProps<SVGSVGElement>) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    fill="none"
    viewBox="0 0 14 14"
    {...props}
  >
    <path
      fill="#606060"
      fillRule="evenodd"
      d="M10.465 2.835a4.9 4.9 0 0 1 0 6.93L7 13.23 3.535 9.765a4.9 4.9 0 0 1 6.93-6.93ZM7 7.7a1.4 1.4 0 1 0 0-2.8 1.4 1.4 0 0 0 0 2.8Z"
      clipRule="evenodd"
    />
  </svg>
);
export default SvgLocationMarker;
