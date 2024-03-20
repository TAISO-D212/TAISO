import { ILayoutProps } from "../types/LayoutProps.ts"

export const MainLayout = ({children}: ILayoutProps) => {
  return (
  <div className="w-[100vw] h-[100vh] flex justify-center items-center">
    {children}
  </div>
  )
}