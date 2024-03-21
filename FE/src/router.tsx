import { createBrowserRouter } from 'react-router-dom';
import { LoginPage } from './pages/login/LoginPage.tsx';
import { MainPage } from './pages/main/MainPage.tsx';
import { Reservation } from './pages/reservation/Reservation.tsx';
import { History } from './pages/history/History.tsx';
import { NotFound } from './pages/NotFound.tsx';
import { UserPage } from './pages/profile/UserPage.tsx';

export const router = createBrowserRouter([
  {path: '/', element: <LoginPage />},
  {path: '/main', element: <MainPage />},
  {path: '/reservation', element: <Reservation />},
  {path: '/history', element: <History />},
  // {path: '/favorite', element: <NotFound />},
  {path: '/profile', element: <UserPage />},
  {path: '/*', element: <NotFound />}
]);
